import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        right_pix = np.sum(Rover.nav_angles <= 0) # the pixels on the right hand side
        left_pix = np.sum(Rover.nav_angles > 0) #  the pixels on the left hand side
        # the pixels in front, implies if there is a obstacle in front
        front_pix = np.sum(Rover.nav_angles < 0.5) + np.sum(Rover.nav_angles > -0.5)
        # the distance between rover and the samples
        rover_sample_dists = np.sqrt((Rover.samples_pos[0] - Rover.pos[0])**2 + \
                            (Rover.samples_pos[1] - Rover.pos[1])**2)
        # the distance between rover and the closest sample
        rover_sample_dists_min = np.min(rover_sample_dists)
        if Rover.mode == 'forward':
            # If in a state where want to pickup a rock send pickup command
            if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
                Rover.send_pickup = True
            # if rover is near sample but still moving, brake!
            elif Rover.near_sample and Rover.vel != 0:
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
            # Check the extent of navigable terrain
            # if rover has place to move and the sample is too far away
            elif ((len(Rover.nav_angles) >= Rover.stop_forward) and \
                ((len(Rover.sample_angles) < 5) or \
                 ((len(Rover.sample_angles) >= 5) and (rover_sample_dists_min >=5)))):
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                if front_pix < Rover.stop_forward + 100:
                    Rover.throttle = -0.1
                    #Rover.brake = Rover.brake_set/50
                    if right_pix >= left_pix:
                        Rover.steer = -15
                    else:
                        Rover.steer = 15
                    # Rover.throttle = Rover.throttle_set
                elif right_pix >= Rover.stop_forward:
                    Rover.steer = np.clip(np.mean(Rover.nav_angles[Rover.nav_angles <= 0.5] * 180/np.pi), -15, 15)
                else:
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi)-4, -15, 15)
            # if rover can move and the sample is not too far away
            elif ((len(Rover.nav_angles) >= Rover.stop_forward) and \
                  (len(Rover.sample_angles) >= 5)):
                # turn to the sample
                Rover.steer = np.clip(np.mean(Rover.sample_angles * 180/np.pi), -15, 15)
                # if rover near sample, stop!
                if (rover_sample_dists_min < 1) or Rover.near_sample:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
                else: # if rover far from sample, move slowly towards the sample
                    if Rover.vel <= 0.2: # if rover stops, move!
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set - 0.3
                    elif Rover.vel >= 1: # if rover move too fast, brake!
                        Rover.throttle = 0
                        Rover.brake = 0.1
                    else: # otherwise keep moving
                        Rover.throttle = 0
                        Rover.brake = 0

                    # Rover.mode = 'stop'
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # if we can pick up the sample, pick up!
                if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
                    Rover.send_pickup = True
                # Now we're stopped and we have vision data to see if there's a path forward
                # if rover close to the obstacles, turn left
                elif (right_pix < Rover.go_forward - 220) and not Rover.near_sample:
                # if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                else:
                # if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0




    return Rover