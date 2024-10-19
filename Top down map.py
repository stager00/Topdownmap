import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
from robot_hat import Ultrasonic, Pin, Music
from picrawler import PiCrawler

# Initialize PiCrawler and Ultrasonic sensor
crawler = PiCrawler()
sonar = Ultrasonic(Pin("D2"), Pin("D3"))
music = Music()

# Initialize the map
map_size = 500  # Size of the map in pixels
map_scale = 10  # Scale factor for converting distances to pixels
obstacle_map = np.zeros((map_size, map_size), dtype=np.uint8)
visited_map = np.zeros((map_size, map_size), dtype=np.uint8)

# Function to measure distance
def measure_distance():
    distance = sonar.read()
    return distance if distance >= 0 else None

# Function to capture camera image
def capture_image():
    camera = cv2.VideoCapture(0)
    ret, frame = camera.read()
    if ret:
        filename = f'camera_image_{int(time.time())}.png'
        cv2.imwrite(filename, frame)
        camera.release()
        return filename
    camera.release()
    return None

# Function to update the map with obstacle data
def update_map(x, y, distance, image_file):
    if distance:
        obstacle_x = int(x + distance * np.cos(np.radians(y)) * map_scale)
        obstacle_y = int(y + distance * np.sin(np.radians(y)) * map_scale)
        if 0 <= obstacle_x < map_size and 0 <= obstacle_y < map_size:
            obstacle_map[obstacle_y, obstacle_x] = 255
            if image_file:
                annotate_map(obstacle_x, obstacle_y, image_file)

# Function to annotate the map with camera images
def annotate_map(x, y, image_file):
    img = Image.open(image_file)
    img.thumbnail((50, 50))  # Resize image to thumbnail
    map_img = Image.fromarray(obstacle_map)
    map_img.paste(img, (x, y))
    obstacle_map[:] = np.array(map_img)

# Function to save the map as a PNG file
def save_map():
    plt.imshow(obstacle_map, cmap='gray')
    plt.title('Top-Down Map of the Area')
    plt.savefig('top_down_map.png')
    plt.show()

# Function to mark visited locations
def mark_visited(x, y):
    visited_map[y, x] = 255

# Function to check if a location is visited
def is_visited(x, y):
    return visited_map[y, x] == 255

# Main loop
alert_distance = 10  # Distance threshold in cm
x, y = map_size // 2, map_size // 2  # Start position in the center of the map

try:
    while True:
        distance = measure_distance()
        if distance and distance <= alert_distance:
            music.sound_play_threading('./sounds/sign.wav', volume=100)
            image_file = capture_image()
            update_map(x, y, distance, image_file)
            mark_visited(x, y)
            crawler.do_action('turn_left_angle', 3)
        else:
            if not is_visited(x, y):
                crawler.do_action('forward', 1)
                x += 1  # Update x position as the robot moves forward
                mark_visited(x, y)
            else:
                crawler.do_action('turn_left_angle', 3)  # Turn to find new area
        time.sleep(0.2)

        # Save the map periodically
        if time.time() % 10 < 0.2:
            save_map()

except KeyboardInterrupt:
    print("Program stopped by User")
    crawler.stop()
    save_map()
