#!/bin/bash

# ========================================
# F450 Drone Simulator - Main Entry Point
# ========================================
# Usage: ./sim.sh [command]
#   ./sim.sh         - Start simulation with GUI
#   ./sim.sh test    - Test motor control
#   ./sim.sh stop    - Stop simulation
#   ./sim.sh help    - Show help
# ========================================

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() { echo -e "${GREEN}✓${NC} $1"; }
print_warning() { echo -e "${YELLOW}⚠${NC} $1"; }
print_error() { echo -e "${RED}✗${NC} $1"; }
print_info() { echo -e "${BLUE}ℹ${NC} $1"; }

# Parse command
COMMAND=${1:-start}

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    print_error "Docker not installed. Please install Docker first."
    exit 1
fi

# Main logic
case $COMMAND in
    start|"")
        print_info "Starting F450 Drone Simulation with GUI..."
        
        # Allow X11 access
        xhost +local:docker 2>/dev/null || true
        
        # Stop any existing simulation
        docker compose -f docker/docker-compose.yml down 2>/dev/null || true
        
        # Run simulation with GUI
        docker run --rm \
            --name f450_simulation \
            -e DISPLAY=$DISPLAY \
            -e QT_X11_NO_MITSHM=1 \
            -e XDG_RUNTIME_DIR=/tmp/runtime-root \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v $(pwd)/src:/workspace/src \
            -v $(pwd)/algorithms:/workspace/algorithms \
            --network host \
            --privileged \
            f450-drone-sim:latest \
            bash -c '
                # Start simulation with grass world
                cd /workspace/PX4-Autopilot
                
                echo ""
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                echo "🚁 F450 Drone Simulation Starting..."
                echo "   • Grass ground for better drone visibility"
                echo "   • Press Ctrl+C to stop"
                echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
                echo ""
                
                # Since PX4 is pre-built, this make will be very fast (just launches)
                echo "Applying grass ground..."
                
                # Replace the default empty world with our grass version
                cp /workspace/src/simulation/worlds/f450_grass.world Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world
                
                # Use PX4s make system with empty world (now grass) - this should be fast since binaries are pre-built
                vglrun -d :0 make px4_sitl gazebo-classic_iris__empty
            '
        ;;
        
    test)
        print_info "Testing Motor Control..."
        
        # Check if simulation is running
        if ! docker ps | grep -q f450_simulation; then
            print_error "Simulation not running! Start it first with: ./sim.sh"
            exit 1
        fi
        
        # Run motor test
        docker exec f450_simulation bash -c "
            source /opt/ros/humble/setup.bash
            cd /workspace
            
            # Build ROS2 package if needed
            if [ ! -d install/drone_control ]; then
                echo 'Building ROS2 packages...'
                colcon build --packages-select drone_control
            fi
            
            source install/setup.bash
            
            # Start MAVROS
            echo 'Connecting to drone...'
            ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557 &
            MAVROS_PID=\$!
            
            sleep 5
            
            # Run motor test
            echo 'Testing motors (watch the drone!)...'
            ros2 run drone_control motor_test_mavros2
            
            kill \$MAVROS_PID 2>/dev/null
        "
        ;;
        
    stop)
        print_info "Stopping simulation..."
        docker stop f450_simulation 2>/dev/null || true
        docker compose -f docker/docker-compose.yml down 2>/dev/null || true
        print_status "Simulation stopped"
        ;;
        
    shell)
        print_info "Entering simulation shell..."
        docker exec -it f450_simulation bash
        ;;
        
    rviz|viz)
        print_info "Starting RViz for depth camera visualization..."
        
        # Check if simulation is running
        if ! docker ps | grep -q f450_simulation; then
            print_error "Simulation not running! Start it first with: ./sim.sh"
            exit 1
        fi
        
        # Allow X11 access for RViz GUI
        xhost +local:docker 2>/dev/null || true
        
        # Launch RViz with MAVROS2 bridge for ROS2 topics
        docker exec -it f450_simulation bash -c "
            export DISPLAY=\$DISPLAY
            export QT_X11_NO_MITSHM=1
            source /opt/ros/humble/setup.bash
            cd /workspace
            
            # Start MAVROS2 to bridge PX4 to ROS2 with TF publishing enabled
            echo 'Starting MAVROS2 bridge with TF publishing enabled...'
            ros2 run mavros mavros_node --ros-args \
                -p fcu_url:=udp://:14540@127.0.0.1:14557 \
                -p local_position.tf.send:=true \
                -p local_position.tf.send_fcu:=false \
                -p local_position.frame_id:=map \
                -p local_position.tf.frame_id:=map \
                -p local_position.tf.child_frame_id:=base_link &
            MAVROS_PID=\$!
            
            # MAVROS now handles TF publishing and pose data directly
            
            # Start robot_state_publisher for the drone model
            echo 'Starting robot state publisher for drone model...'
            ros2 run robot_state_publisher robot_state_publisher \
                --ros-args -p robot_description:=\"\$(cat /workspace/src/simulation/urdf/f450_drone.urdf)\" &
            RSP_PID=\$!
            
            # Wait for MAVROS to start
            sleep 5
            
            # Explicitly enable TF publishing (in case it gets overridden)
            echo 'Ensuring MAVROS TF publishing is enabled...'
            ros2 param set /mavros/local_position tf.send true
            
            # Wait a bit more for TF to start
            sleep 2
            
            echo 'Services started. Checking topics...'
            ros2 topic list | grep -E '(drone_pose|mavros/global)' || echo 'Topics not ready'
            
            # Start RViz with simple test config
            echo 'Starting RViz with test visualization config...'
            echo 'Drone should appear as red arrow near origin'
            rviz2 -d /workspace/src/simulation/rviz/test_pose.rviz
        "
        print_status "RViz started! You should see depth camera point clouds and images."
        print_info "The depth camera is forward-facing on the drone."
        ;;
        
    qgc|qgroundcontrol)
        print_info "Starting QGroundControl GUI..."
        
        # Check if simulation is running
        if ! docker ps | grep -q f450_simulation; then
            print_error "Simulation not running! Start it first with: ./sim.sh"
            exit 1
        fi
        
        # Allow X11 access for GUI
        xhost +local:docker 2>/dev/null || true
        
        # Start QGroundControl with GUI
        docker compose -f docker/docker-compose.yml --profile qgc up -d qgroundcontrol
        
        print_status "QGroundControl GUI started!"
        print_info "QGC interface will open - connects automatically to drone on port 14550"
        print_info "Use QGC for flight planning, monitoring, and manual control"
        ;;
        
    build-image|image)
        print_info "Building Docker image with pre-installed dependencies..."
        cd docker
        docker compose build
        docker compose down 2>/dev/null || true  # Ensure no containers are running
        cd ..
        print_status "Docker image build complete! Includes VirtualGL and pre-built PX4."
        print_info "Image built successfully. Use './sim.sh start' to run simulation."
        ;;
        
    rebuild|fresh)
        print_info "Rebuilding Docker image from scratch (no cache)..."
        cd docker
        docker compose build --no-cache --pull
        docker compose down 2>/dev/null || true  # Ensure no containers are running
        cd ..
        print_status "Docker image rebuilt from scratch! Includes VirtualGL and pre-built PX4."
        print_info "Fresh image built successfully. Use './sim.sh start' to run simulation."
        ;;
        
    clean)
        print_warning "This will remove all containers and images. Continue? (y/N)"
        read -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            docker compose -f docker/docker-compose.yml down --rmi all --volumes
            docker system prune -f
            print_status "Cleanup complete"
        fi
        ;;
        
    help|--help|-h)
        echo "🚁 F450 Drone Simulator"
        echo ""
        echo "Usage: ./sim.sh [command]"
        echo ""
        echo "Commands:"
        echo "  start       Start simulation with GUI (default)"
        echo "  test        Test motor control"
        echo "  rviz        Start RViz for depth camera visualization"
        echo "  qgc         Start QGroundControl GUI"
        echo "  stop        Stop simulation"
        echo "  shell       Enter simulation container"
        echo "  build-image Build Docker image with dependencies"
        echo "  rebuild     Rebuild Docker image from scratch (no cache)"
        echo "  clean       Remove all Docker images and containers"
        echo "  help        Show this help"
        echo ""
        echo "Examples:"
        echo "  ./sim.sh           # Start simulation"
        echo "  ./sim.sh rviz      # Start RViz to see depth camera"
        echo "  ./sim.sh test      # Test motors"
        echo "  ./sim.sh stop      # Stop everything"
        ;;
        
    *)
        print_error "Unknown command: $COMMAND"
        echo "Run './sim.sh help' for usage"
        exit 1
        ;;
esac