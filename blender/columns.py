import bpy
import glob
import csv, copy
import mathutils
import math
import random

def update_camera(camera, location ,focus_point=mathutils.Vector((0.0, 0.0, 0.0)), distance=10.0):
    """
    Focus the camera to a focus point and place the camera at a specific distance from that
    focus point. The camera stays in a direct line with the focus point.
    """
    looking_direction = location - focus_point
    rot_quat = looking_direction.to_track_quat('Z', 'Y')

    camera.rotation_euler = rot_quat.to_euler()
    camera.location = location

def load_roadmap(file):
    samples = []
    edges = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            col = []
            for c in row:
                col.append(float(c))
            if len(row)==3:
                samples.append(col)
            else:
                edges.append(col)
    return samples, edges


def load_trajectory_samples_sst(file):
    print("load_trajectory_samples ",file)
    edges = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        header = next(csvreader)
        last_pos = None
        for row in csvreader:
            col = []
            for c in row:
                col.append(float(c))
            print(col)
            new_pos = [col[3],col[4],col[5]]
            if last_pos is not None:
                edges.append(last_pos + new_pos)    
            last_pos = new_pos
    #print(edges)
    return edges


def load_trajectory_samples_cpc(file):
    print("load_trajectory_samples ",file)
    edges = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        header = next(csvreader)
        last_pos = None
        for row in csvreader:
            col = []
            for c in row:
                col.append(float(c))
            new_pos = [col[1],col[2],col[3]]
            #print("new pos ",new_pos)
            if last_pos is not None:
                edges.append(last_pos + new_pos)    
            last_pos = new_pos
    #print(edges)
    return edges

def load_trajectory_samples_pmm(file,header=True):
    print("load_trajectory_samples ",file)
    edges = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        if header:
            header = next(csvreader)
        last_pos = None
        for row in csvreader:
            col = []
            for c in row:
                col.append(float(c))
            new_pos = [col[1],col[2],col[3]]
            if last_pos is not None:
                edges.append(last_pos + new_pos)    
            last_pos = new_pos
    #print(edges)
    return edges

def plot_curve(edgelist,name, color = (0.0,1.0,0.0,1.0),width=0.01,material=None):
    crv = bpy.data.curves.new('crv', 'CURVE')
    crv.dimensions = '3D'
    spline = crv.splines.new(type='POLY')
    #one point is there already
    spline.points.add(1) 
    edge = edgelist[0]
    spline.points[-2].co = ([edge[0],edge[1],edge[2], 1.0])
    spline.points[-1].co = ([edge[3],edge[4],edge[5], 1.0])
    if material is None:
        material = bpy.data.materials.new(name+"_material")
        material.diffuse_color = color
        crv.materials.append(material)
    else:
        crv.materials.append(material)
    crv.bevel_depth = width
    
    for edgeid in range(1,len(edgelist)):
        edge = edgelist[edgeid]
        #print(edge)
        spline.points.add(2) 
        #print(type(spline.points[-2]))
        spline.points[-2].co = ([edge[0],edge[1],edge[2], 1.0])
        spline.points[-1].co = ([edge[3],edge[4],edge[5], 1.0])

    obj = bpy.data.objects.new(name, crv)
    bpy.data.scenes[0].collection.objects.link(obj)
    
def point_cloud(ob_name, coords, edges=[], faces=[]):
    """Create point cloud object based on given coordinates and name.

    Keyword arguments:
    ob_name -- new object name
    coords -- float triplets eg: [(-1.0, 1.0, 0.0), (-1.0, -1.0, 0.0)]
    """

    # Create new mesh and a new object
    me = bpy.data.meshes.new(ob_name + "Mesh")
    ob = bpy.data.objects.new(ob_name, me)

    # Make a mesh from a list of vertices/edges/faces
    me.from_pydata(coords, edges, faces)

    # Display name and update the mesh
    ob.show_name = True
    me.update()
    return ob

name = 'generated_curve'    
#project = '/home/robert/rpg_workspace/droneracing_planner'
project = '/home/robert/my_workspace/topological_planning'
roadmap_shortened_path_files = glob.glob(project+'/roadmap_shortened_path*.csv')
roadmap_shortened_correct_dir_path_files = glob.glob(project+'/roadmap_shortened_correct_dir_path*.csv')
roadmap_shortened_unique_path_files = glob.glob(project+'/roadmap_shortened_unique_path*.csv')
roadmap_path_files = glob.glob(project+'/roadmap_path*.csv')
roadmap_seeds_cluster_file = project+'/roadmap_seeds_cluster.csv'
roadmap_files = glob.glob(project+'/roadmap_all0.csv')
between_cluster_path_files =  glob.glob(project+'/roadmap_path_cluster*.csv')
trajectory_file_pmm = project+'/samples.csv'
trajectory_file_sst = project+'/path.csv'
trajectory_file_sst_dense = project+'/path_dense.csv'
trajectory_file_polynomial = project+'/polynomial_path.csv'
trajectory_file_polynomial_reference = project+'/shortest_position_path.csv'

cpc_project='/home/robert/rpg_workspace/time_optimal_trajectory'
cpc_trajectory_file=cpc_project+'/results/arena_obst/final.csv'


roadmap_clusters_files = glob.glob(project+'/roadmap_cluster_*.csv')

#remove old generated paths
for model in bpy.data.objects:
    print(model)
    if name in model.name:
        bpy.data.objects.remove(model)

print("after removal")
print("about to load files",roadmap_shortened_path_files)

#trajectory_pmm = load_trajectory_samples_pmm(trajectory_file_pmm)
#trajectory_sst = load_trajectory_samples_sst(trajectory_file_sst)
#trajectory_sst_dense = load_trajectory_samples_pmm(trajectory_file_sst_dense)
#trajectory_polynomial = load_trajectory_samples_pmm(trajectory_file_polynomial)
#trajectory_polynomial_reference = load_trajectory_samples_pmm(trajectory_file_polynomial_reference,False)
#trajectory_cpc = load_trajectory_samples_cpc(cpc_trajectory_file)


#print(trajectory_polynomial_reference)
#print(trajectory_sst)

roadmap_edges = []
roadmap_samples = []
for file in roadmap_files:
    print("loading ",file)
    #    id = int(file.replace("roadmap_cluster_","").replace(".csv",""))
    samples,edges = load_roadmap(file)
    roadmap_edges.append(edges)
    roadmap_samples.append(samples)

cluster_edges = {}
cluster_samples = {}
for file in roadmap_clusters_files:
    print("loading ",file)
    print(file.split("/")[-1].replace("roadmap_cluster_","").replace(".csv",""))
    id = int(file.split("/")[-1].replace("roadmap_cluster_","").replace(".csv",""))
    samples,edges = load_roadmap(file)
    cluster_edges[id] = edges
    cluster_samples[id] = samples

cluster_seed_samples, _ = load_roadmap(roadmap_seeds_cluster_file)

between_cluster_paths = []
for file in between_cluster_path_files:
    samples,edges = load_roadmap(file)
    between_cluster_paths.append(edges)
    
paths = []
for file in roadmap_path_files:
    #print("loading ",file)
    samples,edges = load_roadmap(file)
    paths.append(edges)
        
path_shortened_edges = []
for file in roadmap_shortened_path_files:
    #print("loading ",file)
    samples,edges = load_roadmap(file)
    path_shortened_edges.append(edges)
    
path_shortened_correct_dir_edges = []
for file in roadmap_shortened_correct_dir_path_files:
    samples,edges = load_roadmap(file)
    path_shortened_correct_dir_edges.append(edges)
    
path_shortened_unique_edges = []
for file in roadmap_shortened_unique_path_files:
    #print("loading ",file)
    samples,edges = load_roadmap(file)
    path_shortened_unique_edges.append(edges)

#for path_edges in roadmap_edges:
#    #print(["path_edges ",path_edges)
#    for edge in path_edges:
#        plot_curve([edge],name,color=(1.0,0,0,1.0))

for id in cluster_edges:
    path_edges = cluster_edges[id]

    #print(["path_edges ",path_edges)
    
    mat_name = 'colcl' + str(id)
    mat = bpy.data.materials.get(mat_name)
    if mat is None:
        mat = bpy.data.materials.new(name=mat_name)
        rand_color = (random.random(),random.random(), random.random(),1.0)
        mat.diffuse_color = rand_color
        
        
    print("cl",id)    
    obj_copy = bpy.context.scene.objects['cluster'].copy()
    obj_copy.data = obj_copy.data.copy() # linked = False
    obj_copy.name = 'generated_curve_cl'+str(id)
    bpy.context.collection.objects.link(obj_copy)
    
    obj_copy = bpy.context.scene.objects['generated_curve_cl'+str(id)]  
    endpos = cluster_seed_samples[id]
    print(cluster_seed_samples[id])
    obj_copy.location = mathutils.Vector((endpos[0],endpos[1],endpos[2]))    
    
    obj_copy.data.materials.append(mat)
    for edge in path_edges:
        plot_curve([edge],name,material=mat)
        
        
#for path_edges in between_cluster_paths:
#    for edge in path_edges:
#        plot_curve([edge],name,color = (1.0,0.0,1.0,1.0),width=0.08)
#    
#tst = bpy.ops.mesh.primitive_ico_sphere_add(radius=0.5, location=(0, 0, 1.3)) 
#print("mesh",tst)
#pc = point_cloud("point-cloud", )
#bpy.context.collepction.objects.link(pc)

#for path_edges in paths:
#    for edge in path_edges:
#        plot_curve([edge],name)p


#for path_edges in path_shortened_edges:
#    for edge in path_edges:
#        plot_curve([edge],name)
        
#for path_edges in path_shortened_correct_dir_edges:
#    for edge in path_edges:
#        plot_curve([edge],name)


 
#for path_edges in path_shortened_unique_edges:
#    #print("path_edges ",path_edges)
#    #plot_curve(path_edges,name)
#    for edge in path_edges:
#        plot_curve([edge],name,color=(0,0,0,1.0),width=0.1)


#plot_curve(trajectory_pmm,name+'pmm',color = (1.0,0.0,1.0,1.0),width=0.1)
#plot_curve(trajectory_sst,name+'sst',color = (0.0,1.0,0.0,1.0),width=0.1)
#plot_curve(trajectory_sst_dense,name+'sstdense',color = (0.0,1.0,0.0,1.0),width=0.1)

#plot_curve(trajectory_polynomial,name+'poly',color = (1.0,1.0,0,1.0),width=0.1)
#plot_curve(trajectory_polynomial_reference,name+'polyref',color = (1.0,1.0,1.0,1.0),width=0.1)

#plot_curve(trajectory_cpc,name+'cpc',color = (1.0,0.0,0,1.0),width=0.1)



#remove old generated paths
#for model in bpy.data.objects:
#    print(model)
#    if name in model.name:
#        bpy.data.objects.remove(model)


render = False
if render:
    for i in range(360):
        ang = math.pi * i/180.0
        center = mathutils.Vector((3.0, 0.45, 0.0))
        camera_pos = center + mathutils.Vector((20*math.cos(ang), 20*math.sin(ang), 3.5))
        
        update_camera(bpy.data.objects['Camera'],camera_pos,focus_point=center)
        bpy.context.scene.render.filepath =  '/home/robert/Pictures/poksst/image'+str(i)
        bpy.ops.render.render(write_still = True)
    print("after")