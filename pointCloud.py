# -*- coding: utf-8 -*-
"""
Created on Sun Jul 30 12:35:30 2023

@author: johnk

Resources
http://www.open3d.org/docs/release/tutorial/geometry/pointcloud_outlier_removal.html
"""
import sys
import point_cloud_utils as pcu
import pandas as pd
import numpy as np
import open3d as o3d
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import meshio
from scipy import optimize


class PointCloud:
    def __init__(self, filename = 'trunk30b.ply' ):
       
        self.df = self.ReadPly(filename)
        #self.df = self.Center(self.df)
        #self.df.hist()         
        #miny = -0.2
        #maxy = 0.2
        #self.df = self.Reduce(self.df,'Y',miny,maxy)
        
        #minx = -0.65
        #maxx = 0.89
       # self.df = self.Reduce(self.df,'X',minx,maxx)
       # self.df = self.Reduce(self.df,'Z',minx,maxx)
        
        #self.ExtractByColor()
        #self.df = self.DownSample(self.df,1e5)
        #self.DropNeighborless(0.007)
        #self.DropNeighborless()
        #self.DropNeighborless(0.002)
        self.df.reset_index(drop=True)
        
       # self.df = self.Orient()
         
       # xmin, fval, funcallsint = optimize.golden(self.MinimizeStdX, brack=(-1, 1), full_output=True, maxiter = 11) # goes with one below
       # self.df = self.RotateY(self.df,xmin)                                                                       # goes with one above 
   
        #self.df = self.Center(self.df)
        #self.CutXZPlane()
        
        self.DropNeighborless2()
        #xmin, fval, funcallsint = optimize.golden(self.MinimizeStdY, brack=(-1, 1), full_output=True, maxiter = 11) # goes with one below
        #self.df = self.Rotate(self.df,'Z',xmin)                                                                       # goes with one above 

        self.WritrPly("trunk30c.ply")
        
        self.Visualize_o3d()
        #self.df.hist()
    

        
    
    def CutXZPlane(self):
        self.df = self.df.drop(self.df[self.df['Z'] < 0].index)
        return
    
    def DownSample(self,df,n=1e6):
        # v is a nv by 3 NumPy array of vertices
        v = self.DatFrameToPoints_o3d(df)
        idx = pcu.downsample_point_cloud_poisson_disk(v, num_samples=int(n))
        df = df.loc[df.index[idx]]
        df.reset_index(drop=True)
        return df

    
    def ReadFile_o3d(self)  :
        pcd = o3d.io.read_point_cloud('C:/Users/johnk/Documents/triumph/pointClouds/trunk2.ply')
        return pcd
    
    def DatFrameToPoints_o3d(self, df):
        return df[['X', 'Y', 'Z']].to_numpy()
    
    def DataFrameToPointCloud_o3d(self, df):
        
        o3dPoints = self.DatFrameToPoints_o3d(df)

        pc =o3d.geometry.PointCloud(o3d.cpu.pybind.utility.Vector3dVector(o3dPoints))
        c =  df[['R', 'G', 'B', ]].to_numpy()
        pc.colors = o3d.utility.Vector3dVector(c)
        
        return pc
        
    def ReadPly(self,filename="trunk.ply"):
        path = 'C:/Users/johnk/Documents/triumph/pointClouds/'
        v, f, n, c = pcu.load_mesh_vfnc(path+filename)
        df1 = pd.DataFrame(np.squeeze(v), columns=['X', 'Y', 'Z'])
        df2 = pd.DataFrame(np.squeeze(c), columns=['R', 'G', 'B', 'A'])
        
        df = pd.merge(df1, df2, left_index=True, right_index=True)
        
        return df
        
    def WritrPly(self,filename="trunk2.ply"):
        path = 'C:/Users/johnk/Documents/triumph/pointClouds/'

        #pcu.save_mesh_v(path+filename, v)
        
        v = self.df[['X','Y','Z']].to_numpy()
        c = self.df[['R','G','B','A']].to_numpy()
        #pcu.save_mesh_vfnc("path/to/mash", v, f, n, c)
        
        pcu.save_mesh_vfnc(path+filename, v=v, f=None, n=None,c=c)
        
        
       # self.Visualize_o3d(path+filename)
        

    def MakeVTKMesh(self):
        points = np.array(self.pcd.points)
        tri = Delaunay(points)
        cells = []

        triangles = []
        triangles.append("triangle")
        triangles.append([])            #  only like this because no triangles
        quadfaces = []
        for face in tri.simplices:
            l = len(face)
            quadfaces.append(face)
            if (l < 4):
                print(l)
        
        q =("quad",quadfaces)
        #t=("triangle",[])
        cells.append(q)
        #cells.append(t)

        mesh = meshio.Mesh(points.tolist(),cells)  #https://github.com/nschloe/meshio
        
        meshio.write_points_cells("C:/Users/johnk/Documents/triumph/pointClouds/foo.vtk", points.tolist(), cells)
        

    def Ransac(self):
        return
        
    def Center(self,df):
        df['X'] = df['X'] - df['X'].mean()
        df['Y'] = df['Y'] - df['Y'].mean()
        df['Z'] = df['Z'] - df['Z'].mean()
        return df
    
    def Reduce(self,df,column,minimum,maximum):
        df = df.drop(df[df[column] < minimum].index)
        df = df.drop(df[df[column] > maximum].index)
        return df
    
    def Visualize_o3d(self):
        pcd = self.DataFrameToPointCloud_o3d(self.df)
        pcds = []
        pcds.append(pcd)

        #o3d.io.write_point_cloud("copy_of_fragment.pcd", self.pcd)
        o3d.visualization.draw_geometries(pcds)
        
        
        return
        
    def ExtractByColor(self):  # rid of anythin w/o a lot of red
        self.df.drop(self.df[self.df['R'] <= 0.6].index, inplace = True)
        #self.df.drop(self.df[self.df['G'] > 0.6].index, inplace = True)
        return
    
    def DropNeighborless(self, d = 0.001):
        v = self.df[['X','Y','Z']].to_numpy()
        dists_a_to_b, corrs_a_to_b = pcu.k_nearest_neighbors(v, v, 2)
        self.df['dist'] = (dists_a_to_b.transpose()[1])
        self.df.drop(self.df[self.df['dist'] >= d].index, inplace = True)
        
        
        return
    
    def DropNeighborless2(self, d = 0.004):
        v = self.df[['X','Y','Z']].to_numpy()
        dists_a_to_b, corrs_a_to_b = pcu.k_nearest_neighbors(v, v, 50)
        
        dist_a_to_b_std = []
        for dd in dists_a_to_b:
            dist_a_to_b_std.append(np.std(dd))
            

        self.df['dist_a_to_b_std'] = dist_a_to_b_std
        self.df.drop(self.df[self.df['dist_a_to_b_std'] >= d].index, inplace = True)
        
        
        return
    
    def Rotate(self, df, axis = 'Y', rot = 1):
        
        theta = np.radians(rot)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))
        
        if axis =='Y':
            axis1 = "X"
            axis2 = "Z"
        elif axis == 'Z':
            axis1 = "X"
            axis2 = "Y"
        else:
            axis1 = "Y"
            axis2 = "Z"
                
        
        for index, row in df.iterrows():
            a = np.array([row[axis1], row[axis2]])
            b = np.matmul(R,a)
            df.loc[index,axis1] = b[0]
            df.loc[index,axis2]= b[1];

        #df.hist()
        return df
    
    
    def RotateY(self, df, rot = 1):   # should need this use generic Rotate
        
        theta = np.radians(rot)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))
        
        for index, row in df.iterrows():
            a = np.array([row['X'], row['Z']])
            b = np.matmul(R,a)
            df.loc[index,'X'] = b[0]
            df.loc[index,'Z']= b[1];

        #df.hist()
        return df
    
    def MinimizeStdX(self, rot =1):
        df = self.df.copy()
        self.RotateY(df,rot)
        delta = df['X'].std()   
        print(df['X'].std())
        return delta
    
    def MinimizeStdY(self, rot =1):
        df = self.df.copy()
        
        self.Rotate(df,'Z',rot)
        delta = df['Y'].std()   
        print(df['Y'].std())
        return delta
    
    def FitLines(self):
        return
        

def main(commandLineInput=[]):
    #pc = PointCloud("hood9.ply")
    pc = PointCloud()
    



if __name__ == "__main__":
    main(sys.argv[1:])