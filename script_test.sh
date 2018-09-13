#!/bin/bash

END=2
for i in $(seq 1 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/box/point_cloud_box+rotation_no_$i --object box --number $i
done

for i in $(seq 1 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/car/point_cloud_car+rotation_no_$i --object car --number $i
done

for i in $(seq 1 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/cleanser/point_cloud_cleanser+rotation_no_$i --object cleanser --number $i
done

for i in $(seq 1 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/cylinder/point_cloud_cylinder+rotation_no_$i --object cylinder --number $i
done

for i in $(seq 1 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/toy/point_cloud_toy+rotation_no_$i --object toy --number $i
done

for i in $(seq 1 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/toy/point_cloud_box+rotation_no_$i --object only-trasl --number $i
done

