#!/bin/bash

set -e
set -u

density=20000
radius=0.01

# Create Floor
./pcl_spawn_box --output floor.pcd --count $density
pcl_transform_point_cloud floor.pcd floor.pcd -scale 1,0.1,1
pcl_transform_point_cloud floor.pcd floor.pcd -trans 0,-1,0
./pcl_spawn_uniform3f --output floor_albedo.pcd --count $density --x 0 --y 1 --z 0
./pcl_spawn_uniform3f --output floor_emission.pcd --count $density --x 0 --y 0 --z 0

# Create Left Wall
./pcl_spawn_box --output left.pcd --count $density
pcl_transform_point_cloud left.pcd left.pcd -scale 0.1,1,1
pcl_transform_point_cloud left.pcd left.pcd -trans -1,0,0
./pcl_spawn_uniform3f --output left_albedo.pcd --count $density --x 0 --y 1 --z 0
./pcl_spawn_uniform3f --output left_emission.pcd --count $density --x 0 --y 0 --z 0

# Create Back Wall
./pcl_spawn_box --output back.pcd --count $density
pcl_transform_point_cloud back.pcd back.pcd -scale 1,1,0.1
pcl_transform_point_cloud back.pcd back.pcd -trans 0,0,-1
./pcl_spawn_uniform3f --output back_albedo.pcd --count $density --x 0 --y 1 --z 0
./pcl_spawn_uniform3f --output back_emission.pcd --count $density --x 0 --y 0 --z 0


# Create Light
./pcl_spawn_box --output light.pcd --count 100
pcl_transform_point_cloud light.pcd light.pcd -scale 0.1,0.1,0.1
pcl_transform_point_cloud light.pcd light.pcd -trans 0,0.5,0
./pcl_spawn_uniform3f --output light_albedo.pcd --count $density --x 0 --y 0 --z 0
./pcl_spawn_uniform3f --output light_emission.pcd --count $density --x 5 --y 5 --z 5

pcl_concatenate_points_pcd floor.pcd left.pcd back.pcd light.pcd
mv output.pcd all_points.pcd

pcl_concatenate_points_pcd floor_albedo.pcd left_albedo.pcd back_albedo.pcd light_albedo.pcd
mv output.pcd all_albedo.pcd

pcl_concatenate_points_pcd floor_emission.pcd left_emission.pcd back_emission.pcd light_emission.pcd
mv output.pcd all_emission.pcd

pcl_transform_point_cloud all_points.pcd all_points.pcd -trans 0,0,-3
./pcl_render --points all_points.pcd --albedo all_albedo.pcd --emission all_emission.pcd --spp 1024 --width 1280 --height 720 --radius $radius
