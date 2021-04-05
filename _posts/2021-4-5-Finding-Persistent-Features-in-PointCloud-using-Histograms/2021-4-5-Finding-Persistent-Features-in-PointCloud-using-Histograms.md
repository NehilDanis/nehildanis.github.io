---
layout: post
title: Finding Persistent Features in a PointCloud using Histograms
---

Previously, I had published a blog post about ICP algorithm and its basic usage. Iterative closest point algorith is very 
sensitive to the initial alignment. The good initial alignment will help the algorithm to reach the result faster while 
preventing it from being stuck in a local minima. With a good initial alignment ICP algorithm can even reach the global 
minima.

To find a good initial alignment, one first needs to decide on good features in the source and target cloud. There are 
many points in a point cloud and not all of them are very informative. The informative features can be used to find the 
correspondences between source and target cloud which then lead to the initial guess on transformation between these two 
clouds.

The informative points can be described as any point which will give you enough information about the local and global 
place of the point. If you consider a white wall, there are many points on it. However, 
it is impossible to find the correspondences of these points on the wall in the target point cloud, because it is not distinctive.
On the other hand, a corner can be a good feature point.

### Summary of Persistent Point Feature Histograms for 3D Point Clouds[1]

The two of the most used point features can be estimated surface curvatures and normals for a point. Each point is characterized 
by using their k closest neighbors. Surface curvature and normal estimation of the points are highly sensitive to noise and 
selection of the k neighbors. If data includes large noise, then selecting a small k will lead to large errors, if the k is too big 
then the local details will get lost.

There are algorithms which uses single and multi-scale features. In this paper authors used multi scale feature representations 
of the points. Multi scale is better because even though the single scale feature descriptors can handle the noise in the 
data, since they only have one scale, many points in the dataset will have similar feature vectors, which will reduce their 
informative characteristics. This will make finding correspondences between source and target clouds hard.

In this paper, histograms are used to represent each point in the point cloud. Points have different histogram for each of 
the scale values. Usage of histogram is good because it can encode the neighborhood's geometrical properties while providing 
an overall scale and pose invariant feature.

#### The Steps

Each scale value will represent different radius.

* for each scale value;
    * For each point p, select all the neighbors enclosed in the sphere with given radius.
    * in case the normal of point p is missing, use all the neighbors to estimate it. PCA can be 
    one way to estimate the normal.
    * After all normals are obtained, use the viewpoint to re-orient all the points in the neigborhood consistently.
    * For every pair of points in the neighborhood

### References
[1] Rusu, Radu Bogdan, et al. "Persistent point feature histograms for 3D point clouds." Proc 10th Int Conf Intel 
Autonomous Syst (IAS-10), Baden-Baden, Germany. 2008.



