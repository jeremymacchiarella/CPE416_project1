import rclpy
from rclpy.node import Node

# Needed imports
from math import cos, sin, isfinite, isinf, isnan
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion



class LocalCostmap(Node):
    def __init__(self):
        super().__init__('local_costmap')

        # Map config
        self.map_width = 300            # cells
        self.map_height = 300           # cells
        self.map_resolution = 0.05      # resolution * cells  => 15 m x 15 m map

        # Occupancy conventions
        self.UNKNOWN = -1
        self.FREE = 0
        self.OCCUPIED = 100

        # Outgoing OccupancyGrid message (we'll fill .info and .header)
        self.publish_map = OccupancyGrid()
        self._init_map_info()

        # Precompute robot's cell (center of grid)
        self.cx = self.map_width // 2
        self.cy = self.map_height // 2

        self.scan_data = LaserScan()
        # Subscriber to LaserScan
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        # Publisher for the occupancy grid and initialize data array so indexing is safe
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.publish_map.data = [self.UNKNOWN] * (self.map_width * self.map_height)

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.build_occupancy_grid()


    ''' Initialize static OccupancyGrid.info and origin so the robot is at the map center. '''
    def _init_map_info(self):
        self.publish_map.info.resolution = self.map_resolution
        self.publish_map.info.width = self.map_width
        self.publish_map.info.height = self.map_height

        self.publish_map.header.frame_id = 'diff_drive/odom'


        # Place (0,0) of the grid so that the robot (base frame origin) is at the center cell.
        # That means the map origin (bottom-left corner in world coords) is shifted negative by half-size.
        origin = Pose()
        origin.position.x = - (self.map_width * self.map_resolution) / 2.0
        origin.position.y = - (self.map_height * self.map_resolution) / 2.0
        origin.position.z = 0.0
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.publish_map.info.origin = origin

    ''' Meters in robot frame -> map indices (mx, my). Returns None if out of bounds. '''
    # Input: (x, y) coordinates of a point in the Cartesian plane
    # Output: Corresponding cell in the occupancy grid
    def world_to_map(self, x_m, y_m):
        mx = int((x_m - self.publish_map.info.origin.position.x) / self.publish_map.info.resolution)
        my = int((y_m - self.publish_map.info.origin.position.y) / self.publish_map.info.resolution)

        # clamp if it is out of bounds
        mx = max(0, min(mx, self.map_width - 1))
        my = max(0, min(my, self.map_height - 1))
        return mx, my

    # Bresenham's Line Algorithm: inclusive endpoints
    # Input: 2-points on the Cartesian plane (i.e. a line)
    # (The first point is the robot origin, while the sencond is a single beam's endpoint)
    # Output: All the cells that that the beam crosses. i.e. the free cells.
    def bresenham_line_algorithm(self, x0, y0, x1, y1): 
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy  

        free_space_cells = []
        while True:
            free_space_cells.append((x0, y0))
            e2 = 2 * err
            if (e2 >= dy):
                if (x0 == x1):
                    break
                err += dy
                x0 += sx
            if (e2 <= dx):
                if (y0 == y1):
                    break
                err += dx
                y0 += sy

        return free_space_cells
    
    ''' Cache the most recent LaserScan'''
    def laser_callback(self, msg: LaserScan):

        self.get_logger().info('reading scan data')
        self.scan_data = msg

        
        
        
        
        return

    # Input: x & y coordinates;
    # Output: list of free cells along the ray (excludes the last cell)
    def raytrace(self, x_cell, y_cell):
        # Compute free cells for a single beam
        # This function should call self.bresenham_line_algorithm
        free_cells = self.bresenham_line_algorithm(self.cx, self.cy, x_cell, y_cell)
        # exclude last one bc it is occupied??
        free_cells = free_cells[:-1]

        return free_cells

    ''' Build and Publish the Occupancy Grid from the most recent LiDAR Scan '''
    def build_occupancy_grid(self):

        #   MAKE SURE THE OCCUPANCY GRID IS INITIALIZED TO ALL UNKNOWN
        # First, check that the scan data is ready

        # index = y * width + x

        # create a temporary grid initialized to UNKNOWN and populate it
        grid = [self.UNKNOWN] * self.map_width * self.map_height

        robot_points = self.transform_lidar_to_robot()
        for point in robot_points:
            hit = point[2] # flag for if beam did not hit anything (nothing in range)
            # need to convert robot coordinate to cell in map
            map_cell = self.world_to_map(point[0], point[1]) 

            # here still need to trace to make all the free space marked as free
            if (map_cell is None):
                continue # dont care about this one 
            mx = map_cell[0]
            my = map_cell[1]

            free_cells = self.raytrace(mx, my)

            for cell in free_cells:
                cx = cell[0]
                cy = cell[1]
                if (0 <= cx < self.map_width and 0 <= cy < self.map_height):
                    # idx in occupancy grid is y * width + x
                    idx = cy * self.map_width + cx
                    grid[idx] = self.FREE
            
            if (0 <= mx < self.map_width and 0 <= my < self.map_height and hit == True):
                idx = my * self.map_width + mx
                grid[idx] = self.OCCUPIED


        # cpy temp grid to actual grid 
        self.publish_map.data = grid
        self.publish_map.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info('publishing map data')
        
        self.publisher_.publish(self.publish_map)

    def transform_lidar_to_robot(self):
        # this function will transform the lidar coordinates to robot coordinates
        x_lidar = 0.1
        y_lidar = -0.1
        theta_lidar = 0.0

        robot_points = []


        # need to select which i values actually matter
        max_r = self.scan_data.range_max
        for i,r in enumerate(self.scan_data.ranges):
            hit = True
            if isinf(r):
                r = max_r
                hit = False
            if isnan(r):
                continue #invalid reading
            
            
            angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
             
            # get point in robot frame 
            # negative 1 to account for robot center to edge of robot
            x_robot = x_lidar + r * cos(angle + theta_lidar)
            y_robot = y_lidar + r * sin(angle + theta_lidar)

            robot_points.append((x_robot, y_robot, hit))
        return robot_points


def main(args=None):
    rclpy.init(args=args)
    
    # Node creation and spin
    local_costmap = LocalCostmap()
    rclpy.spin(local_costmap)
    
    # Node cleanup
    local_costmap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
