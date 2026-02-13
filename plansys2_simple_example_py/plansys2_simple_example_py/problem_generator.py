#!/usr/bin/env python3

## @package spot_apf
# 
# \file ray.py
# \brief this node casts rays to retreive obstacles position
#
# author Zoe Betta
# version 1.0
# date 09/05/2025
# \details
# 
# Subscribes to: <BR>
# 
#
# Publishes to: <BR>
#    /obstacle_marker
#
# Client: <BR>
# 
#
# Action Service: <BR>
# 
#
# Parameters: <BR>
# angle: angle of the rays
# 
# Description: <BR>
#  This node connects to the spot robot and, using its SDK it casts ray every "angle" degrees to collect the position of the obstacles around the robot in the body frame
# 
import argparse
import numpy as np


def get_age():
    while True:
        age_input = input("Enter your age: ")
        if age_input.isdigit():
            return int(age_input)
        else:
            print("Please enter a valid number.")

def main():
    seed = input("Enter the seed number for randomness: ")
    while not seed.isdigit():
        print("Please enter a valid number.")
        seed = input("Enter the seed number for randomness: ")
    floor = input("Enter the number of floors: ")
    while not floor.isdigit():
        print("Please enter a valid number.")
        floor = input("Enter the number of floors: ")
    roomxfloor = input("Enter the number of rooms x floor: ")
    while not roomxfloor.isdigit():
        print("Please enter a valid number.")
        roomxfloor = input("Enter the number of rooms x floor: ")
    blockedlinks = input("Enter the percentage of blocked links: ")
    while not blockedlinks.isdigit():
        print("Please enter a valid number.")
        blockedlinks = input("Enter the percentage of blocked links: ") 
    initialposition = input("Enter the initial position (room number ex 2a if room 2 of floor 1), if 0 random start: ")
    closeddoors = input("Enter the percentage of closed doors: ")
    while not closeddoors.isdigit():
        print("Please enter a valid number.")
        closeddoors = input("Enter the percentage of closed doors: ")
    blockeddoors = input("Enter the percentage of blocked doors: ")
    while not blockeddoors.isdigit():
        print("Please enter a valid number.")
        blockeddoors = input("Enter the percentage of blocked doors: ")
    people = input("Enter the number of people in the environment: ")
    while not people.isdigit():
        print("Please enter a valid number.")
        people = input("Enter the number of people in the environment: ")
    emergency= input("Is there an emergency? (y/n), if 0 random: ")
    while emergency.lower() not in ['y', 'n', '0']:
        print("Please enter 'y', 'n', or '0'.")
        emergency = input("Is there an emergency? (y/n) if 0 random: ")
    visited_rooms = input("Enter the percentage of visited rooms: ")
    while not visited_rooms.isdigit():
        print("Please enter a valid number.")
        visited_rooms = input("Enter the percentage of visited rooms: ")
    broken_stairs = input("Enter the percentage of broken stairs: ")
    while not broken_stairs.isdigit():
        print("Please enter a valid number.")
        broken_stairs = input("Enter the percentage of broken stairs: ")   
    


    # COME CAZZO METTO LE EMERGENZE? CHE PARTO GIA' IN EMERGENZA? perchè in realtà è una cosa improvvisa, quindi o parto così o niente



    print("\n--- Summary ---")
    print(f"Seed: {seed}")
    print(f"Floor: {floor}")
    print(f"Rooms per floor: {roomxfloor}")
    print(f"Blocked links: {blockedlinks}%")
    print(f"Initial position: {initialposition}")
    print(f"Closed doors: {closeddoors}%")
    print(f"Blocked doors: {blockeddoors}%")
    print(f"People: {people}")
    print(f"Emergency: {emergency}")
    print(f"Visited rooms: {visited_rooms}%")
    print(f"Broken stairs: {broken_stairs}%")
    

if __name__ == "__main__":
    main()
