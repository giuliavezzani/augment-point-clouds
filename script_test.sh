#!/bin/bash

END=0
for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/only-trasl/box/point_cloud_box_trasl+rotation_no_$i --object box_trasl --number $i
done

for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/only-trasl/car/point_cloud_car_trasl+rotation_no_$i --object car_trasl --number $i
done

for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/only-trasl/cleanser/point_cloud_cleanser_only_trasl+rotation_no_$i --object cleanser_trasl --number $i
done

for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/only-trasl/cylinder/point_cloud_cylinder+rotation_no_$i --object cylinder_trasl --number $i
done

for i in $(seq 0 $END); do
	superq-and-grasp-visualizer --hand both --get_grasping_pose --remove-outliers "(0.01 10)" --file data/only-trasl/toy/point_cloud_toy+rotation_no_$i --object toy_trasl --number $i
done



