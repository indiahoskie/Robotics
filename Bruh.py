# Constants
TARGET_DISTANCE = 50  # Target distance in cm
KP = 0.5  # Proportional gain

# Simulated sensor reading (distance to target in cm)
current_distance = 100

# Control loop
while True:
    # Calculate error
    error = TARGET_DISTANCE - current_distance

    # Calculate control signal (speed)
    speed = KP * error

    # Clamp speed to a maximum/minimum value
    speed = max(min(speed, 100), -100)

    # Simulate robot movement (update current distance)
    current_distance -= speed * 0.1  # Adjust factor for simulation

    # Print status
    print(f"Error: {error}, Speed: {speed}, Current Distance: {current_distance}")

    # Break condition for simulation
    if abs(error) < 1:
        print("Target reached!")
        break

