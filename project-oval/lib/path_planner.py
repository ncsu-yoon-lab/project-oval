import json
import math
import heapq
from collections import defaultdict
from typing import List, Tuple, Dict, Optional

class PathPlanner:
    """
    A pathfinding class for navigation networks with lat/lon coordinates converted to metric system.
    
    This class handles:
    - Loading and processing navigation data from JSON
    - Converting lat/lon coordinates to X/Y meters relative to an origin
    - Building a graph structure for pathfinding
    - Finding shortest paths between points using Dijkstra's algorithm
    - Finding closest points to arbitrary coordinates
    """
    
    def __init__(self, json_data: Dict = None, json_file_path: str = None):
        """
        Initialize the PathPlanner with navigation data.
        
        Args:
            json_data (Dict, optional): Pre-loaded JSON data dictionary
            json_file_path (str, optional): Path to JSON file to load
            
        Raises:
            ValueError: If neither json_data nor json_file_path is provided
            FileNotFoundError: If json_file_path doesn't exist
        """
        if json_data is None and json_file_path is None:
            raise ValueError("Either json_data or json_file_path must be provided")
        
        # Load data
        if json_data is not None:
            self.data = json_data
        else:
            self.data = self._load_json_file(json_file_path)
        
        # Initialize data structures
        self.nodes: Dict[int, Dict] = {}
        self.points: Dict[int, Dict] = {}
        self.graph: Dict[int, List[Tuple[int, float]]] = defaultdict(list)
        self.segments: set = set()
        self.origin_lat: float = None
        self.origin_lon: float = None
        
        # Process the data
        self._process_data()
        self._build_graph()
    
    def _load_json_file(self, file_path: str) -> Dict:
        """Load JSON data from file."""
        try:
            with open(file_path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"JSON file not found: {file_path}")
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON file: {e}")
    
    def _latlon_to_meters(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Convert lat/lon coordinates to X/Y meters relative to origin.
        
        Args:
            lat (float): Latitude in degrees
            lon (float): Longitude in degrees
            
        Returns:
            Tuple[float, float]: (x_meters, y_meters) relative to origin
        """
        if self.origin_lat is None or self.origin_lon is None:
            return 0.0, 0.0
        
        # Constants for conversion
        LAT_TO_METERS = 111320.0  # meters per degree latitude
        
        # Longitude conversion varies by latitude
        lon_to_meters = LAT_TO_METERS * math.cos(math.radians(self.origin_lat))
        
        # Calculate relative position in meters
        x_meters = (lon - self.origin_lon) * lon_to_meters
        y_meters = (lat - self.origin_lat) * LAT_TO_METERS
        
        return x_meters, y_meters
    
    def _meters_to_latlon(self, x_meters: float, y_meters: float) -> Tuple[float, float]:
        """
        Convert X/Y meters to lat/lon coordinates.
        
        Args:
            x_meters (float): X coordinate in meters
            y_meters (float): Y coordinate in meters
            
        Returns:
            Tuple[float, float]: (latitude, longitude) in degrees
        """
        if self.origin_lat is None or self.origin_lon is None:
            return 0.0, 0.0
        
        # Constants for conversion
        LAT_TO_METERS = 111320.0  # meters per degree latitude
        lon_to_meters = LAT_TO_METERS * math.cos(math.radians(self.origin_lat))
        
        # Calculate lat/lon from meters
        lat = self.origin_lat + (y_meters / LAT_TO_METERS)
        lon = self.origin_lon + (x_meters / lon_to_meters)
        
        return lat, lon
    
    def _process_data(self):
        """Process the loaded JSON data and convert coordinates."""
        if not self.data:
            return
        
        # Get origin point
        origin = self.data.get('origin', [35.771192, -78.674462])
        self.origin_lat = origin[0]
        self.origin_lon = origin[1]
        
        # Process nodes
        for node in self.data.get('nodes', []):
            x, y = self._latlon_to_meters(node['latlon'][0], node['latlon'][1])
            self.nodes[node['id']] = {
                'name': node['node_name'],
                'lat': node['latlon'][0],
                'lon': node['latlon'][1],
                'x': x,
                'y': y,
                'type': 'node'
            }
        
        # Process points
        for point in self.data.get('points', []):
            x, y = self._latlon_to_meters(point['latlon'][0], point['latlon'][1])
            self.points[point['id']] = {
                'lat': point['latlon'][0],
                'lon': point['latlon'][1],
                'x': x,
                'y': y,
                'linked_points': point['linked_points'],
                'segment_num': point['segment_num'],
                'type': 'point'
            }
            self.segments.add(point['segment_num'])
    
    def _build_graph(self):
        """Build adjacency graph for pathfinding using metric distances."""
        self.graph = defaultdict(list)
        
        # Add connections from points data
        for point_id, point_data in self.points.items():
            for linked_id in point_data['linked_points']:
                # Get the linked point/node data
                if linked_id in self.points:
                    other_point = self.points[linked_id]
                elif linked_id in self.nodes:
                    other_point = self.nodes[linked_id]
                else:
                    continue
                
                # Calculate distance in meters
                distance = self._calculate_distance_meters(
                    point_data['x'], point_data['y'],
                    other_point['x'], other_point['y']
                )
                
                # Add bidirectional edges
                self.graph[point_id].append((linked_id, distance))
                self.graph[linked_id].append((point_id, distance))
    
    def _calculate_distance_meters(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate Euclidean distance between two points in meters."""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def find_closest_point(self, x: float, y: float) -> Optional[int]:
        """
        Find the closest point or node to given X/Y coordinates.
        
        Args:
            x (float): X coordinate in meters
            y (float): Y coordinate in meters
            
        Returns:
            Optional[int]: ID of closest point/node, or None if no points exist
        """
        min_distance = float('inf')
        closest_id = None
        
        # Check all points
        for point_id, point_data in self.points.items():
            distance = self._calculate_distance_meters(x, y, point_data['x'], point_data['y'])
            if distance < min_distance:
                min_distance = distance
                closest_id = point_id
        
        # Check all nodes
        for node_id, node_data in self.nodes.items():
            distance = self._calculate_distance_meters(x, y, node_data['x'], node_data['y'])
            if distance < min_distance:
                min_distance = distance
                closest_id = node_id
        
        return closest_id
    
    def _dijkstra(self, start: int, end: int) -> List[int]:
        """
        Find shortest path using Dijkstra's algorithm.
        
        Args:
            start (int): Starting point/node ID
            end (int): Destination point/node ID
            
        Returns:
            List[int]: List of point/node IDs representing the path, empty if no path found
        """
        # Initialize distances and previous nodes
        all_ids = list(self.points.keys()) + list(self.nodes.keys())
        distances = {node_id: float('inf') for node_id in all_ids}
        distances[start] = 0
        previous = {}
        heap = [(0, start)]
        visited = set()
        
        while heap:
            current_distance, current = heapq.heappop(heap)
            
            if current in visited:
                continue
            
            visited.add(current)
            
            # Found destination
            if current == end:
                break
            
            # Check all neighbors
            for neighbor, weight in self.graph[current]:
                distance = current_distance + weight
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current
                    heapq.heappush(heap, (distance, neighbor))
        
        # Reconstruct path
        if end not in previous and start != end:
            return []  # No path found
        
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = previous.get(current)
        
        return path[::-1]  # Reverse to get path from start to end
    
    def plan_path(self, current_x: float, current_y: float, destination_id: int) -> Dict:
        """
        Plan a path from current position to destination.
        
        Args:
            current_x (float): Current X position in meters
            current_y (float): Current Y position in meters
            destination_id (int): ID of destination point/node
            
        Returns:
            Dict: Path planning result containing:
                - success (bool): Whether path was found
                - path (List[int]): List of point/node IDs representing the path
                - distance (float): Total path distance in meters
                - start_point_id (int): ID of closest point to current position
                - message (str): Human-readable result message
                
        Raises:
            ValueError: If destination_id doesn't exist
        """
        # Validate destination
        if destination_id not in self.points and destination_id not in self.nodes:
            raise ValueError(f"Destination ID {destination_id} not found in the network")
        
        # Find closest point to current position
        closest_start = self.find_closest_point(current_x, current_y)
        
        if closest_start is None:
            return {
                'success': False,
                'path': [],
                'distance': 0.0,
                'start_point_id': None,
                'message': 'No points found in the network'
            }
        
        # Find path using Dijkstra
        path = self._dijkstra(closest_start, destination_id)
        
        if not path:
            return {
                'success': False,
                'path': [],
                'distance': 0.0,
                'start_point_id': closest_start,
                'message': f'No path found from point {closest_start} to {destination_id}'
            }
        
        # Calculate total distance
        total_distance = 0.0
        for i in range(len(path) - 1):
            current_id = path[i]
            next_id = path[i + 1]
            
            # Find distance between consecutive points in path
            for neighbor, distance in self.graph[current_id]:
                if neighbor == next_id:
                    total_distance += distance
                    break
        
        return {
            'success': True,
            'path': path,
            'distance': total_distance,
            'start_point_id': closest_start,
            'message': f'Path found: {len(path)} points, {total_distance:.1f}m total distance'
        }
    
    def get_point_info(self, point_id: int) -> Optional[Dict]:
        """
        Get information about a specific point or node.
        
        Args:
            point_id (int): ID of the point/node
            
        Returns:
            Optional[Dict]: Point/node information, or None if not found
        """
        if point_id in self.points:
            return self.points[point_id].copy()
        elif point_id in self.nodes:
            return self.nodes[point_id].copy()
        else:
            return None
    
    def get_path_coordinates(self, path: List[int]) -> List[Tuple[float, float]]:
        """
        Get X/Y coordinates for all points in a path.
        
        Args:
            path (List[int]): List of point/node IDs
            
        Returns:
            List[Tuple[float, float]]: List of (x, y) coordinates in meters
        """
        coordinates = []
        for point_id in path:
            if point_id in self.points:
                point_data = self.points[point_id]
                coordinates.append((point_data['x'], point_data['y']))
            elif point_id in self.nodes:
                node_data = self.nodes[point_id]
                coordinates.append((node_data['x'], node_data['y']))
        return coordinates
    
    def get_available_destinations(self) -> Dict[int, str]:
        """
        Get all available destination points and nodes.
        
        Returns:
            Dict[int, str]: Dictionary mapping IDs to names/descriptions
        """
        destinations = {}
        
        # Add all points
        for point_id in self.points:
            destinations[point_id] = f"Point {point_id}"
        
        # Add all nodes with their names
        for node_id, node_data in self.nodes.items():
            destinations[node_id] = f"Node {node_id} ({node_data['name']})"
        
        return destinations
    
    def get_segments(self) -> set:
        """Get all available segment numbers."""
        return self.segments.copy()
    
    def get_origin_coordinates(self) -> Tuple[float, float]:
        """Get the origin coordinates in lat/lon."""
        return self.origin_lat, self.origin_lon