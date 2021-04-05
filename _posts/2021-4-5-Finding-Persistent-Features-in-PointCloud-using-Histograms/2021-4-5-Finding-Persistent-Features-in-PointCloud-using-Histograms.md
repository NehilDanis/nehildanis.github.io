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

There are many different ways to find good feature points, such as edge detectors. Today I will talk about finding the features 
using histograms[1].

### References
[1] Rusu, Radu Bogdan, et al. "Persistent point feature histograms for 3D point clouds." Proc 10th Int Conf Intel 
Autonomous Syst (IAS-10), Baden-Baden, Germany. 2008.



