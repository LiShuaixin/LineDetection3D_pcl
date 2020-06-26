#ifndef PCL_OCTREE_POINTCLOUD_ADJACENCY_TOOL_H_
#define PCL_OCTREE_POINTCLOUD_ADJACENCY_TOOL_H_

#include <pcl/octree/boost.h>
#include <pcl/octree/octree.h>

#include <set>
#include <list>

using namespace pcl::octree;

namespace pcl
{
  namespace octree
  {
    class LeafNodeKey : public OctreeKey
    {
    public:
	int index_;
	
    public:
	LeafNodeKey() : OctreeKey()
	              , index_(0)
        {}
        
        LeafNodeKey (unsigned int keyX, unsigned int keyY, unsigned int keyZ) : OctreeKey(keyX, keyY, keyZ)
        {
	    index_ = 4*keyZ + 2*keyY + 1*keyX;
        }
        
        LeafNodeKey (const LeafNodeKey& source) : index_(source.index_)
	{
	    memcpy(this->key_, source.key_, sizeof(key_));
	}
	
	LeafNodeKey (const OctreeKey& source)
	{
	    memcpy(this->key_, source.key_, sizeof(key_));
	    update();
	}
	
	bool operator == (const LeafNodeKey& b) const
	{
	    return ((b.x == this->x) && (b.y == this->y) && (b.z == this->z));
	}
	
	bool operator <= (const LeafNodeKey& b) const
	{
	    return ((b.x >= this->x) && (b.y >= this->y) && (b.z >= this->z));
	}
	
	bool operator >= (const LeafNodeKey& b) const
	{
	    return ((b.x <= this->x) && (b.y <= this->y) && (b.z <= this->z));
	}
	
	bool operator < (const LeafNodeKey& b) const
	{
	   if (b.z > this->z)
	       return true;
	   else if (b.z == this->z && b.y > this->y)
	       return true;
	   else if (b.z == this->z && b.y == this->y && b.x > this->x)
	       return true;
	   else 
	       return false;
	}
	
	bool operator > (const LeafNodeKey& b) const
	{
	    if (b.z < this->z)
	       return true;
	   else if (b.z == this->z && b.y < this->y)
	       return true;
	   else if (b.z == this->z && b.y == this->y && b.x < this->x)
	       return true;
	   else 
	       return false;
	}
	
	void update()
	{
	    index_ = 4*x + 2*y + 1*z;
	}
    };

    template <typename PointT, typename LeafContainerT = OctreeContainerPointIndices,
	      typename BranchContainerT = OctreeContainerEmpty>
    class OctreePointCloudVoxel : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT>
    {
    public:

	typedef OctreePointCloudVoxel<PointT, LeafContainerT, BranchContainerT> OctreeVoxelT;
	typedef boost::shared_ptr<OctreeVoxelT> Ptr;
	typedef boost::shared_ptr<const OctreeVoxelT> ConstPtr;
		
	/** \brief Constructor.
          *
          * \param[in] resolution_arg Octree resolution at lowest octree level (voxel size) */
	OctreePointCloudVoxel (const double resolution_arg) : OctreePointCloud< PointT, LeafContainerT, BranchContainerT
                                                                              , OctreeBase<LeafContainerT, BranchContainerT> > (resolution_arg)
	{
	}

	/** \brief Empty class destructor. */
	virtual ~OctreePointCloudVoxel ()
	{
	}

	/** \brief Generates octree key for specified point (uses transform if provided).
          *
          * \param[in] point_arg Point to generate key for
          * \param[out] key_arg Resulting octree key */
        void
        genOctreeKeyforPoint (const PointT& point_arg, OctreeKey& key_arg) const
        {
	    if (transform_func_)
	    {
		PointT temp (point_arg);
		transform_func_ (temp);
	        // calculate integer key for transformed point coordinates
		if (pcl::isFinite (temp)) //Make sure transformed point is finite - if it is not, it gets default key
		{
		    key_arg.x = static_cast<unsigned int> ((temp.x - this->min_x_) / this->resolution_);
		    key_arg.y = static_cast<unsigned int> ((temp.y - this->min_y_) / this->resolution_);
		    key_arg.z = static_cast<unsigned int> ((temp.z - this->min_z_) / this->resolution_);
		}
		else
		{
		    key_arg = OctreeKey ();
		}
	    }
	    else 
	    {
		// calculate integer key for point coordinates
		key_arg.x = static_cast<unsigned int> ((point_arg.x - this->min_x_) / this->resolution_);
		key_arg.y = static_cast<unsigned int> ((point_arg.y - this->min_y_) / this->resolution_);
		key_arg.z = static_cast<unsigned int> ((point_arg.z - this->min_z_) / this->resolution_);
	    }
	}
	
	/** \brief Gets the leaf container for a given point.
          *
          * \param[in] point_arg Point to search for
          *
          * \returns Pointer to the leaf container - null if no leaf container found. */
        LeafContainerT* getLeafContainerAtPoint (const PointT& point_arg) const
        {
	    OctreeKey key;
	    LeafContainerT* leaf = 0;
	    // generate key
	    this->genOctreeKeyforPoint (point_arg, key);
	    
	    leaf = this->findLeaf (key);
	    
	    return leaf;
	}
        
        /** \brief Gets the leaf containers for a given voxel key.
          *
          * \param[in] voxel_key Key of voxel to search
          *
          * \returns Vector of pointers to the neighbour leaf containers except the central voxel - null if no leaf container found. */
        std::vector<pcl::octree::LeafNodeKey> getNeighborVoxelKeysAtVoxel (const pcl::octree::LeafNodeKey& voxel_key) const
        {
	    std::vector<pcl::octree::LeafNodeKey> neighbor_voxel_keys;
	    for (int x = voxel_key.x-1; x<voxel_key.x+2; ++x)
	    {
		if (x < 0)
		    continue;
		
		for (int y = voxel_key.y-1; y<voxel_key.y+2; ++y)
		{
		    if (y < 0)
		        continue;
		    
		    for (int z = voxel_key.z-1; z<voxel_key.z+2; ++z)
		    {
			if (z < 0)
		            continue;
			
			pcl::octree::LeafNodeKey neighbor_voxel_key(x, y, z);
			if (this->existLeaf (neighbor_voxel_key))
			    neighbor_voxel_keys.push_back(neighbor_voxel_key);
		    }
		}
	    }
	    
	    return neighbor_voxel_keys;
	}
	
	void getOctreeKeyforPoint(const PointT& point_arg, LeafNodeKey& key_arg) const
	{
	    
	    OctreeKey key_arg_tmp;
	    this->genOctreeKeyforPoint(point_arg, key_arg_tmp);
	    // std::cout << "point: ---- x = " << point_arg.x << ", y = " << point_arg.y << ", z = " << point_arg.z << std::endl;
	    // std::cout << "index: ---- x = " << key_arg_tmp.x << ", y = " << key_arg_tmp.y << ", z = " << key_arg_tmp.z << std::endl;
	    
	    key_arg = LeafNodeKey(key_arg_tmp);
	}
	
	/** \brief Sets a point transform (and inverse) used to transform the space of the input cloud.
          *
          * This is useful for changing how adjacency is calculated - such as relaxing the adjacency criterion for
          * points further from the camera.
          *
          * \param[in] transform_func A boost:function pointer to the transform to be used. The transform must have one
          * parameter (a point) which it modifies in place. */
        void
        setTransformFunction (boost::function<void (PointT &p)> transform_func)
        {
          transform_func_ = transform_func;
        }
        
        /** \brief Tests whether input point is occluded from specified camera point by other voxels.
          *
          * \param[in] point_arg Point to test for
          * \param[in] camera_pos Position of camera, defaults to origin
          *
          * \returns True if path to camera is blocked by a voxel, false otherwise. */
        bool
        testForOcclusion (const PointT& point_arg, const PointXYZ &camera_pos = PointXYZ (0, 0, 0))
	{
	    OctreeKey key;
	    this->genOctreeKeyforPoint (point_arg, key);
	    // This code follows the method in Octree::PointCloud
	    Eigen::Vector3f sensor(camera_pos.x,
				    camera_pos.y,
				    camera_pos.z);
	    
	    Eigen::Vector3f leaf_centroid(static_cast<float> ((static_cast<double> (key.x) + 0.5f) * this->resolution_ + this->min_x_),
					    static_cast<float> ((static_cast<double> (key.y) + 0.5f) * this->resolution_ + this->min_y_), 
					    static_cast<float> ((static_cast<double> (key.z) + 0.5f) * this->resolution_ + this->min_z_));
	    Eigen::Vector3f direction = sensor - leaf_centroid;
	    
	    float norm = direction.norm ();
	    direction.normalize ();
	    float precision = 1.0f;
	    const float step_size = static_cast<const float> (this->resolution_) * precision;
	    const int nsteps = std::max (1, static_cast<int> (norm / step_size));
	    
	    OctreeKey prev_key = key;
	    // Walk along the line segment with small steps.
	    Eigen::Vector3f p = leaf_centroid;
	    PointT octree_p;
	    for (int i = 0; i < nsteps; ++i)
	    {
		//Start at the leaf voxel, and move back towards sensor.
		p += (direction * step_size);
		
		octree_p.x = p.x ();
		octree_p.y = p.y ();
		octree_p.z = p.z ();
		//  std::cout << octree_p<< "\n";
		OctreeKey key;
		this->genOctreeKeyforPoint (octree_p, key);
		
		// Not a new key, still the same voxel (starts at self).
		if ((key == prev_key))
		continue;
		
		prev_key = key;
		
		LeafContainerT *leaf = this->findLeaf (key);
		//If the voxel is occupied, there is a possible occlusion
		if (leaf)
		{
		    return true;
		}
	    }
	    
	    //If we didn't run into a voxel on the way to this camera, it can't be occluded.
	    return false;
	}
	
	///< cotinuous voxel deviding \todo
	
        /// parameters ///
    private:
	boost::function<void (PointT &p)> transform_func_;
    };
  }
}
#endif // PCL_OCTREE_POINTCLOUD_ADJACENCY_TOOL_H_

