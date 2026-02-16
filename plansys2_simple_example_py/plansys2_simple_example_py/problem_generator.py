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
import math
import random

def get_age():
    while True:
        age_input = input("Enter your age: ")
        if age_input.isdigit():
            return int(age_input)
        else:
            print("Please enter a valid number.")

def main():
    problem_number = input("Enter the number of problems to generate: ")
    while not problem_number.isdigit():
        print("Please enter a valid number.")
        problem_number = input("Enter the number of problems to generate: ")
    seed = input("Enter the seed number for randomness: ")
    while not seed.isdigit():
        print("Please enter a valid number.")
        seed = input("Enter the seed number for randomness: ")
    floor = input("Enter the number of floors: ")
    while not floor.isdigit():
        print("Please enter a valid number.")
        floor = input("Enter the number of floors: ")
    roomxfloor = input("Enter the number of rooms x floor (4, 9, 16, 25): ")
    while not roomxfloor.isdigit():
        print("Please enter a valid number.")
        roomxfloor = input("Enter the number of rooms x floor: ")
    blockedlinks = input("Enter the percentage of blocked links: ")
    while not blockedlinks.isdigit():
        print("Please enter a valid number.")
        blockedlinks = input("Enter the percentage of blocked links: ") 
    initialposition = input("Enter the initial position (room number ex r11f3 if room of floor 4), if 0 random start: ")
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
    
    rng = np.random.default_rng(seed=int(seed))
    random.seed(int(seed)) 
    goal = []
    goal.append("and")
    for i in range(int(problem_number)):

        print(f"\nGenerating problem {i+1}...")
        rows=int(math.sqrt(int(roomxfloor)))
        problem_file= f"problem_{i+1}.pddl"
        goal_file= f"goal_{i+1}.txt"
        file = open(problem_file, "w")
        #file_goal = open(goal_file, "w")
        file.write(f"(define (problem spot_problem_{i+1})\n")
        file.write("  (:domain spot_domain)\n\n")
        objects=[]
        init=[]
        goal=[]
        #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"spot", "robot"}});\n')
        objects.append("spot - robot")
        #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"stand", "state"}});\n')
        #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"lay", "state"}});\n')
        #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"sit", "state"}});\n')
        objects.append("stand lay sit unknown - state")
        #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"unknown", "state"}});\n')
        #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"conscious", "consciousness"}});\n')
        #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"unconscious", "consciousness"}});\n')
        #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"confused", "consciousness"}});\n')
        objects.append("conscious unconscious confused - consciousness")
        #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"exit", "location"}});\n')
        objects.append("exit - location")
        #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(is_exit exit)"));\n')
        init.append("(is_exit exit)")
        init.append("(is_free spot)")
        #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(is_free spot)"));\n')

        rooms= np.empty((int(floor), int(rows), int(rows)), dtype=object)
        for j in range(int(floor)):
            if j!=0:
                #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"s1{j}", "stairs"}});\n')
                objects.append("s1"+str(j)+" - stairs")
                objects.append("s2"+str(j)+" - stairs")
                #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"s2{j}", "stairs"}});\n')
            for k in range(int(rows)):
                for l in range(int(rows)):
                    rooms[j][k][l] = "r"+str(k)+str(l)+"f"+str(j)
                    name="r"+str(k)+str(l)+"f"+str(j)
                    #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"{name}", "location"}});\n')
                    objects.append(name+" - location")
        # dove parte il robot
        start= rng.choice(rooms.flatten()) if initialposition=="0" else initialposition
        #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(robot_at spot {start})"));\n')
        init.append("(robot_at spot "+start+")")
        if emergency=="0":
            emergency= rng.choice(['y', 'n'])
        if emergency=="y":
            #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(emergency spot)"));\n')
            init.append("(emergency spot)")
        else:
            #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(not_emergency spot)"));\n')
            init.append("(not_emergency spot)")
        # connessioni
        number_link= int(rows)*(int(rows)-1)*2*int(floor)
        blocked_link_number= int(number_link*int(blockedlinks)/100)
        block_link= random.sample(range(0, number_link), blocked_link_number)
        #print(block_link)
        counter=0
        for i in range(int(floor)):
            for j in range(int(rows)):
                for k in range(int(rows)):
                    if j!=int(rows)-1:
                        if counter not in block_link:
                            #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(connected {rooms[i][j+1][k]} {rooms[i][j][k]})"));\n')
                            init.append("(connected "+rooms[i][j+1][k]+" "+rooms[i][j][k]+")")
                            init.append("(connected "+rooms[i][j][k]+" "+rooms[i][j+1][k]+")")
                            #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(connected {rooms[i][j][k]} {rooms[i][j+1][k]})"));\n')
                        counter+=1
                    if k!=int(rows)-1:
                        if counter not in block_link:
                            init.append("(connected "+rooms[i][j][k+1]+" "+rooms[i][j][k]+")")
                            init.append("(connected "+rooms[i][j][k]+" "+rooms[i][j][k+1]+")")
                            #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(connected {rooms[i][j][k+1]} {rooms[i][j][k]})"));\n')
                            #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(connected {rooms[i][j][k]} {rooms[i][j][k+1]})"));\n')
                        counter+=1
        #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(connected exit {rooms[0][0][0]})"));\n')
        init.append("(connected exit "+rooms[0][0][0]+")")
        init.append("(connected "+rooms[0][0][0]+" exit)")
        #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(connected {rooms[0][0][0]} exit)"));\n')
        # doors blocked and closed
        number_rooms= int(rows)*int(rows)*int(floor)
        blocked_doors_number= int(number_rooms*int(blockeddoors)/100)
        closed_doors_number= int(number_rooms*int(closeddoors)/100)
        block_close= random.sample(range(0, number_rooms), blocked_doors_number+closed_doors_number)
        block= block_close[:blocked_doors_number]
        close= block_close[blocked_doors_number:]
        people_where=random.sample(range(0, number_rooms), int(people))
        for i in range(int(people)):
            #file.write(f'problem_expert_->addInstance(plansys2::Instance{{"p{i+1}", "person"}});\n')
            objects.append("p"+str(i+1)+" - person")
        counter=0
        people_counter=1
        visited_rooms_number= int(number_rooms*int(visited_rooms)/100)
        visited_rooms_list= random.sample(range(0, number_rooms), visited_rooms_number)
        for i in range(int(floor)):
            for j in range(int(rows)):
                for k in range(int(rows)):
                    if counter in block:
                        #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(door_blocked {rooms[i][j][k]})"));\n')
                        init.append("(door_blocked "+rooms[i][j][k]+")")
                    elif counter in close:
                        #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(door_closed {rooms[i][j][k]})"));\n')
                        init.append("(door_closed "+rooms[i][j][k]+")")
                    else:
                        #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(door_notchecked {rooms[i][j][k]})"));\n')
                        init.append("(door_notchecked "+rooms[i][j][k]+")")
                    if counter in people_where:
                        #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(person_detected p{people_counter} {rooms[i][j][k]})"));\n')
                        init.append("(person_detected p"+str(people_counter)+" "+rooms[i][j][k]+")")
                        if counter not in block:
                            goal.append("(person_evaluated p"+str(people_counter)+") (person_reported p"+str(people_counter)+") (dialog_finished p"+str(people_counter)+") ")
                            
                        people_counter+=1
                    if counter not in visited_rooms_list:
                        goal.append("(searched spot "+rooms[i][j][k]+") (environment_checked "+rooms[i][j][k]+") ")
                    counter+=1
        stairs_number= (int(floor)-1)*2
        broken_stairs_number= int(stairs_number*int(broken_stairs)/100)
        broken_stairs_list= random.sample(range(0, stairs_number), broken_stairs_number)
        counter=0
        for i in range(1, int(floor)):
            if counter not in broken_stairs_list:
                #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(stairs_connected {rooms[i-1][0][0]} s1{i})"));\n')
                init.append("(stairs_connected "+rooms[i-1][0][0]+" s1"+str(i)+")")
                #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(stairs_connected {rooms[i][0][0]} s1{i})"));\n')
                init.append("(stairs_connected "+rooms[i][0][0]+" s1"+str(i)+")")
            counter+=1
            if counter not in broken_stairs_list:
                #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(stairs_connected {rooms[i-1][0][int(rows)-1]} s2{i})"));\n')
                init.append("(stairs_connected "+rooms[i-1][0][int(rows)-1]+" s2"+str(i)+")")
                init.append("(stairs_connected "+rooms[i][0][int(rows)-1]+" s2"+str(i)+")")
                #file.write(f'problem_expert_->addPredicate(plansys2::Predicate("(stairs_connected {rooms[i][0][int(rows)-1]} s2{i})"));\n')
            counter+=1
        file.write("(:objects\n")
        for obj in objects:
            file.write(f"  {obj}\n")
        file.write(")\n\n")
        file.write("(:init\n")
        for ini in init:
            file.write(f"  {ini}\n")
        file.write(")\n\n")
        file.write("(:goal\n")
        file.write("(and\n")
        for go in goal:
            file.write(f"  {go}\n")
        file.write(")\n")
        file.write(")")
        file.write(")")

        #goal_str= "".join(goal)
        #file_goal.write(goal_str)
        file.close()
        #file_goal.close()

    print(rooms)

        # Here you would add the code to generate the problem based on the input parameters
        # For example, you could create a function that takes these parameters and generates a problem instance
        # generate_problem(seed, floor, roomxfloor, blockedlinks, initialposition, closeddoors, blockeddoors, people, emergency, visited_rooms, broken_stairs)


    print("\n--- Summary ---")
    print(f"Number of problems: {problem_number}")
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
