include Mk/libs_flags.mk

# OBJS=timer.o common.o base_map.o esdf_map.o pc_map.o blender_loader.o dijkstra.o mesh_object.o prm.o topological_prm.o topological_prm_clustering.o raptor.o pdr.o main.o
# TARGET=main

OBJS=timer.o common.o base_map.o esdf_map.o pc_map.o dijkstra.o prm.o topological_prm.o topological_prm_clustering.o raptor.o pdr.o main.o
TARGET=main

# OBJS=timer.o common.o esdf_map.o drone.o topological_prm.o polynomial_race_test.o
# TARGET=polynomial_race_test

OBJ_DIR=obj

include Mk/recipe.mk
