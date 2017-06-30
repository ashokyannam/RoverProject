import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    if rgb_thresh[2] == 160: # for path
        thresh = (img[:,:,0] > rgb_thresh[0]) \
                    & (img[:,:,1] > rgb_thresh[1]) \
                    & (img[:,:,2] > rgb_thresh[2])
    elif rgb_thresh[2] == 161: # for obstacles
        thresh = ((img[:,:,0] < rgb_thresh[0]) \
                    | (img[:,:,1] < rgb_thresh[1]) \
                    | (img[:,:,2] < rgb_thresh[2]))
    elif rgb_thresh[2] < 100: # for samples
        thresh = ((img[:,:,0] > rgb_thresh[0]) \
                    & (img[:,:,1] > rgb_thresh[1]) \
                    & (img[:,:,2] < rgb_thresh[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select[thresh] = 1
    # Return the binary image
    return color_select

# def color_obstacles(img, rgb_thresh=(160, 160, 160)):
#     below_thresh = np.int_((img[:,:,0] <= rgb_thresh[0]) \
#                 | (img[:,:,1] <= rgb_thresh[1]) \
#                 | (img[:,:,2] <= rgb_thresh[2]))
#     return below_thresh
#
# def color_rock(img, rgb_thresh=(120, 90, 80)):
#     rock_img = np.int_((img[:,:,0] > rgb_thresh[0]) \
#                 & (img[:,:,1] > rgb_thresh[1]) \
#                 & (img[:,:,2] < rgb_thresh[2]))
#     return rock_img

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale))
    # Return the result
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image

    return warped

# get rock position from the rover centric reference
# which is the pixel that closest to the rover
def rock_pos(x, y):
    if len(x) == 0 or len(y) == 0:
        return x, y
    else:
        idx = np.argmin(x)
        return x[idx], y[idx]
#
# def rock_dense(x, y):
#     if len([x]) == 0 or len([y]) == 0:
#         return x, y
#     else:
#         return [x, x, x, x+1, x+1, x+1, x-1, x-1, x-1], [y, y-1, y+1, y, y-1, y+1, y, y-1, y+1]

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                              [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                              [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset]])
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrain = color_thresh(warped, rgb_thresh=(160, 160, 160))
    obstacles = color_thresh(warped, rgb_thresh=(161, 161, 161))
    rock = color_thresh(warped, rgb_thresh=(120, 90, 80))
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles * 200
    Rover.vision_image[:,:,1] = rock * 200
    Rover.vision_image[:,:,2] = terrain * 200
    # 5) Convert map image pixel values to rover-centric coords
    xpix_t, ypix_t = rover_coords(terrain) # terrain coords in rover-centric(rc) reference frame(ref)
    xpix_o, ypix_o = rover_coords(obstacles) # obstacles coordinate in rc ref
    # x_r, y_r = rover_coords(rock)
    xpix_r, ypix_r = rover_coords(rock)

    # x_r, y_r = rock_pos(xpix_r, ypix_r) # sample coordinate in rc ref
    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    world_size = Rover.worldmap.shape[0]
    x_world_t, y_world_t = pix_to_world(xpix_t, ypix_t, xpos, ypos, yaw, world_size, scale)
    # obstacles coords in world ref
    x_world_o, y_world_o = pix_to_world(xpix_o, ypix_o, xpos, ypos, yaw, world_size, scale)
    # rock coords in world ref
    # x_world_r, y_world_r = pix_to_world(xpix_r, ypix_r, xpos, ypos, yaw, world_size, scale)
    # x_r, y_r = pix_to_world(xpix_r, ypix_r, xpos, ypos, yaw, world_size, scale)
    # x_world_r, y_world_r = rock_dense(x_r, y_r)
    x_world_r, y_world_r = pix_to_world(xpix_r, ypix_r, xpos, ypos, yaw, world_size, scale)
    # sample world coords
    # x_world_sample, y_world_sample = pix_to_world(x_r, y_r, xpos, ypos, yaw, world_size, scale)
    # Rover.samples_pos = (x_world_sample, y_world_sample)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[y_world_o, x_world_o, 0] += 1
    Rover.worldmap[y_world_r, x_world_r, 1] += 1
    Rover.worldmap[y_world_t, x_world_t, 2] += 1
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_t, ypix_t)
    Rover.sample_dists, Rover.sample_angles = to_polar_coords(xpix_r, ypix_r)
    # Rover.sample_dists /= scale
    return Rover