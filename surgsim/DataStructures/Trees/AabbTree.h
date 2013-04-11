#pragma once
#ifndef __AABBTREE_H__
#define __AABBTREE_H__

#include "AABB.h"
#include <vector>
#include <set>
#include <utility>
#include "DataStructures/SmallIntMap.h"

class AABBTree
{
	struct ObjectData
	{
		int m_userId;
		double m_center[3];
		_AABB_ m_aabb;

		void setCenter()
		{
			for (int ii = 0; ii < 3; ++ii)
				m_center[ii] = (m_aabb.min[ii]+m_aabb.max[ii])/2 ;
		};

	};

public:
	/// Default constructor
	AABBTree()
		: m_maxNumObjectsInBox(3), m_extraExpansion(0), m_internalBoundingBox(NULL){} ;

	AABBTree(int maxNumObjectsInBox)
		: m_maxNumObjectsInBox(maxNumObjectsInBox), m_extraExpansion(0), m_internalBoundingBox(NULL){} ;

	/// Default destructor
	~AABBTree()
	{
		delete m_internalBoundingBox ;
		m_internalBoundingBox = NULL ;
	} ;

	void setExtraExpansion(double expansion)
	{
		m_extraExpansion = expansion;
	}

	void constructInternalBoundingBoxTree()
	{
		delete m_internalBoundingBox ;
		m_internalBoundingBox = new InternalBoundingBox ;
		std::vector<int> obj_ids;
		for( int ii = 0; ii < (int)m_objects.size(); ++ii )
			obj_ids.push_back(ii) ;
		m_internalBoundingBox->constructBox( m_objects, obj_ids, m_maxNumObjectsInBox ) ;
	}

	int getTotalNumberOfObjects() const { return (int)m_objects.size(); }

	// return the object id inside this class for the added data
	int addObject(int userId, const _AABB_& aabb) 
	{
		m_objects.push_back(ObjectData());
		m_objects.back().m_userId = userId ;
		m_objects.back().m_aabb = aabb ;
		m_objects.back().setCenter();
		m_userIdToObjId[userId] = (int)m_objects.size() - 1;
		return m_userIdToObjId[userId];
	}

	void updateOneObject(int userId, const _AABB_& aabb, bool bUpdateTree = false) 
	{
		if (m_objects.empty()) return;
		if (! m_userIdToObjId.exists(userId)) return;

		m_objects[m_userIdToObjId[userId]].m_aabb = aabb ;
		m_objects[m_userIdToObjId[userId]].setCenter();

		if (bUpdateTree && m_internalBoundingBox)
		{
			m_internalBoundingBox->updateOneObject(m_objects, m_userIdToObjId[userId]);
		}
	}

	void updateInternalBoundingBoxTree() 
	{
		if (m_internalBoundingBox == 0) constructInternalBoundingBoxTree();
		else m_internalBoundingBox->updateAllObjects(m_objects);
	}

	// get the object id inside this class for the specified user id
	int getObjectIdFromUserId(int userId) const
	{
		return m_userIdToObjId[userId];
	}

	const _AABB_& getAABBForOneObjectUsingUserID(int userId) const
	{
		return m_objects[getObjectIdFromUserId(userId)].m_aabb;
	}

	const _AABB_& getAABBForOneObjectUsingObjectID(int objectId) const
	{
		return m_objects[objectId].m_aabb;
	}

	

protected:	

	class InternalBoundingBox
	{
	public:
		InternalBoundingBox()
		{
			m_child1 = m_child2 = NULL ;
		};

		~InternalBoundingBox()
		{ 
			delete m_child1; 
			delete m_child2;
			m_child1 = m_child2 = NULL ;
		};

		InternalBoundingBox(const InternalBoundingBox &rhs)
			: m_child1(0), m_child2(0), m_obj_ids(rhs.m_obj_ids), m_obj_id_set(rhs.m_obj_id_set)
		{
			for (int ii = 0; ii < 3; ++ii)
			{
				m_center[ii] = rhs.m_center[ii] ;
				m_dimension[ii] = rhs.m_dimension[ii] ;
			}
			if (rhs.m_child1) m_child1 = new InternalBoundingBox(*rhs.m_child1);
			if (rhs.m_child2) m_child2 = new InternalBoundingBox(*rhs.m_child2);
		}

		const InternalBoundingBox& operator=(const InternalBoundingBox &rhs)
		{
			if (this != &rhs)
			{
				m_obj_ids = rhs.m_obj_ids;
				m_obj_id_set = rhs.m_obj_id_set;
				for (int ii = 0; ii < 3; ++ii)
				{
					m_center[ii] = rhs.m_center[ii] ;
					m_dimension[ii] = rhs.m_dimension[ii] ;
				}
				delete m_child1; m_child1 = 0; 
				delete m_child2; m_child2 = 0;

				if (rhs.m_child1) m_child1 = new InternalBoundingBox(*rhs.m_child1);
				if (rhs.m_child2) m_child2 = new InternalBoundingBox(*rhs.m_child2);
			}

			return *this;
		}

		bool updateOneObject( const std::vector< ObjectData > &objs, int objId )
		{
			// if it is not a valid box, no need to compute
			if( m_obj_ids.empty() && m_child1 == NULL && m_child2 == NULL )
				return false;

			double min_corner[3], max_corner[3] ;
			int ii;
			if( m_obj_ids.empty() )
			{
				bool flag1 = false, flag2 = false;
				if (m_child1) flag1 = m_child1->updateOneObject( objs, objId ) ;
				if (m_child2) flag2 = m_child2->updateOneObject( objs, objId ) ;
				if (!flag1 && !flag2) return false;

				if( m_child1 )
				{
					for (ii = 0; ii < 3; ++ii)
					{
						min_corner[ii] = m_child1->m_center[ii] - m_child1->m_dimension[ii] ;
						max_corner[ii] = m_child1->m_center[ii] + m_child1->m_dimension[ii] ;
					}

					if( m_child2 )
						for (ii = 0; ii < 3; ++ii)
						{
							if( min_corner[ii] > m_child2->m_center[ii] - m_child2->m_dimension[ii] )
								min_corner[ii] = m_child2->m_center[ii] - m_child2->m_dimension[ii] ;
							if( max_corner[ii] < m_child2->m_center[ii] + m_child2->m_dimension[ii] )
								max_corner[ii] = m_child2->m_center[ii] + m_child2->m_dimension[ii] ;
						}
				}
				else
				{
					for (ii = 0; ii < 3; ++ii)
					{
						min_corner[ii] = m_child2->m_center[ii] - m_child2->m_dimension[ii] ;
						max_corner[ii] = m_child2->m_center[ii] + m_child2->m_dimension[ii] ;
					}
				}

			}
			else
			{
				if (m_obj_id_set.find(objId) == m_obj_id_set.end()) return false;

				for (ii = 0; ii < 3; ++ii)
				{
					min_corner[ii] = 1e18;
					max_corner[ii] = -1e18;
				}
				std::vector<int>::const_iterator itr_obj = m_obj_ids.begin() ;
				for( ; itr_obj != m_obj_ids.end(); ++itr_obj )
				{
					for (ii = 0; ii < 3; ++ii)
					{
						if( min_corner[ii] > objs[*itr_obj].m_aabb.min[ii] )
							min_corner[ii] = objs[*itr_obj].m_aabb.min[ii] ;
						if( max_corner[ii] < objs[*itr_obj].m_aabb.max[ii] )
							max_corner[ii] = objs[*itr_obj].m_aabb.max[ii] ;
					}
				}
			}

			for (ii = 0; ii < 3; ++ii)
			{
				m_center[ii] = (min_corner[ii] + max_corner[ii] ) / 2 ;
				m_dimension[ii] = (max_corner[ii] - min_corner[ii] ) / 2 ;
			}
			return true;
		}

		void updateAllObjects( const std::vector< ObjectData > &objs )
		{
			// if it is not a valid box, no need to compute
			if( m_obj_ids.empty() && m_child1 == NULL && m_child2 == NULL )
				return ;

			double min_corner[3], max_corner[3] ;
			int ii;
			if( m_obj_ids.empty() )
			{
				if( m_child1)
					m_child1->updateAllObjects( objs ) ;
				if( m_child2)
					m_child2->updateAllObjects( objs ) ;

				if( m_child1 )
				{
					for (ii = 0; ii < 3; ++ii)
					{
						min_corner[ii] = m_child1->m_center[ii] - m_child1->m_dimension[ii] ;
						max_corner[ii] = m_child1->m_center[ii] + m_child1->m_dimension[ii] ;
					}

					if( m_child2 )
						for (ii = 0; ii < 3; ++ii)
						{
							if( min_corner[ii] > m_child2->m_center[ii] - m_child2->m_dimension[ii] )
								min_corner[ii] = m_child2->m_center[ii] - m_child2->m_dimension[ii] ;
							if( max_corner[ii] < m_child2->m_center[ii] + m_child2->m_dimension[ii] )
								max_corner[ii] = m_child2->m_center[ii] + m_child2->m_dimension[ii] ;
						}
				}
				else
				{
					for (ii = 0; ii < 3; ++ii)
					{
						min_corner[ii] = m_child2->m_center[ii] - m_child2->m_dimension[ii] ;
						max_corner[ii] = m_child2->m_center[ii] + m_child2->m_dimension[ii] ;
					}
				}

			}
			else
			{
				for (ii = 0; ii < 3; ++ii)
				{
					min_corner[ii] = 1e18;
					max_corner[ii] = -1e18;
				}
				std::vector<int>::const_iterator itr_obj = m_obj_ids.begin() ;
				for( ; itr_obj != m_obj_ids.end(); ++itr_obj )
				{
					for (ii = 0; ii < 3; ++ii)
					{
						if( min_corner[ii] > objs[*itr_obj].m_aabb.min[ii] ) min_corner[ii] = objs[*itr_obj].m_aabb.min[ii] ;
						if( max_corner[ii] < objs[*itr_obj].m_aabb.max[ii] ) max_corner[ii] = objs[*itr_obj].m_aabb.max[ii] ;
					}
				}
			}

			for (ii = 0; ii < 3; ++ii)
			{
				m_center[ii] = (min_corner[ii] + max_corner[ii] ) / 2 ;
				m_dimension[ii] = (max_corner[ii] - min_corner[ii] ) / 2 ;
			}
		}


		void constructBox( const std::vector< ObjectData > &objs, const std::vector<int>& obj_ids, int max_objs )
		{
			int ii ;
			double min_corner[3], max_corner[3];

			// find the bounding box
			for (ii = 0; ii < 3; ++ii)
			{
				min_corner[ii] = 1e18;
				max_corner[ii] = -1e18;
			}
			std::vector<int>::const_iterator itr_obj = obj_ids.begin() ;
			for( ; itr_obj != obj_ids.end(); ++itr_obj )
			{
				for( ii = 0; ii < 3; ++ii )
				{
					if( min_corner[ii] > objs[*itr_obj].m_aabb.min[ii] ) min_corner[ii] = objs[*itr_obj].m_aabb.min[ii] ;
					if( max_corner[ii] < objs[*itr_obj].m_aabb.max[ii] ) max_corner[ii] = objs[*itr_obj].m_aabb.max[ii] ;
				}
			}

			for (ii = 0; ii < 3; ++ii)
			{
				m_center[ii] = (min_corner[ii] + max_corner[ii] ) / 2 ;
				m_dimension[ii] = (max_corner[ii] - min_corner[ii] ) / 2 ;
			}

			delete m_child1 ; m_child1 = NULL ;
			delete m_child2 ; m_child2 = NULL ;
			m_obj_ids.clear() ;

			if( (int)obj_ids.size() <= max_objs )
			{
				m_obj_ids = obj_ids ;
				m_obj_id_set.clear();
				for (int ii = 0; ii < (int)m_obj_ids.size(); ++ii) m_obj_id_set.insert(m_obj_ids[ii]) ;
				return ;
			}

			// need to divide the box into a smaller size
			// find the major axis
			int major_axis = 0 ;
			double major_dim = m_dimension[0] ;
			if( major_dim < m_dimension[1] )
			{
				major_axis = 1 ;
				major_dim = m_dimension[1] ;
			}
			if( major_dim < m_dimension[2] )
			{
				major_axis = 2 ;
				major_dim = m_dimension[2] ;
			}

			std::vector<int> obj_id1, obj_id2 ;
			for( ii = 0; ii < 3; ++ii )
			{
				obj_id1.clear() ;
				obj_id2.clear() ;
				int axis = major_axis + ii ;
				if( axis > 2 ) axis -= 3 ;
				divideBox( axis, objs, obj_ids, obj_id1, obj_id2 ) ;
				if( !obj_id1.empty() && !obj_id2.empty() )
					break ;
			}

			if( obj_id1.empty() || obj_id2.empty() )
			{
				m_obj_ids = obj_ids ;
				return ;
			}

			// create children and construct them
			m_child1 = new InternalBoundingBox ;
			m_child2 = new InternalBoundingBox ;
			m_child1->constructBox( objs, obj_id1, max_objs ) ;
			m_child2->constructBox( objs, obj_id2, max_objs ) ;
		}

		void divideBox( int major_axis, const std::vector< ObjectData > &objs, 
			const std::vector<int>& obj_ids,
			std::vector<int>& index1, std::vector<int>& index2 )
		{

			std::vector<int>::const_iterator itr_obj = obj_ids.begin() ;
			double center = m_center[major_axis] ;
			for( ; itr_obj != obj_ids.end(); ++itr_obj )
			{
				if( objs[*itr_obj].m_center[major_axis] <= center )
					index1.push_back(*itr_obj) ;
				else
					index2.push_back(*itr_obj) ;
			}
		}

		bool intersect( const InternalBoundingBox& other, const double& tolerance ) const
		{
			for (int ii = 0; ii < 3; ++ii)
			{
				if (m_center[ii]+m_dimension[ii]+tolerance < other.m_center[ii]-other.m_dimension[ii]) return false;
				if (m_center[ii]-m_dimension[ii]-tolerance > other.m_center[ii]+other.m_dimension[ii]) return false;
			}
			return true;
		}

		bool intersect( const _AABB_& other, const double& tolerance ) const
		{
			for (int ii = 0; ii < 3; ++ii)
			{
				if (m_center[ii]+m_dimension[ii]+tolerance < other.min[ii]) return false;
				if (m_center[ii]-m_dimension[ii]-tolerance > other.max[ii]) return false;
			}
			return true;
		}

		const size_t getSizeInMemory()
		{
			size_t res=0;
			res += sizeof(m_center);
			res += sizeof(m_dimension);
			res += sizeof(m_child1);
			if(m_child1)
				res += m_child1->getSizeInMemory();
			res += sizeof(m_child2);
			if(m_child2)
				res += m_child2->getSizeInMemory();
			res += sizeof(int) * m_obj_ids.capacity();
			int i = sizeof(m_obj_ids);
			int j = sizeof(int) * m_obj_ids.capacity();
			res += sizeof(m_obj_id_set);
			return res;
		}

		double m_center[3] ;
		double m_dimension[3] ;
		InternalBoundingBox *m_child1 ;
		InternalBoundingBox *m_child2 ;
		std::vector<int> m_obj_ids;
		std::set<int> m_obj_id_set;
	};


public:
	// copy constructor and assignment operator undefined.
	AABBTree( const AABBTree& obj )
		: m_maxNumObjectsInBox(obj.m_maxNumObjectsInBox)
		, m_userIdToObjId(obj.m_userIdToObjId)
		, m_objects(obj.m_objects)
		, m_internalBoundingBox( 0 )
	{
		if (obj.m_internalBoundingBox) m_internalBoundingBox = new InternalBoundingBox(*obj.m_internalBoundingBox) ;
	}

	AABBTree& operator=( const AABBTree & obj )
	{
		m_maxNumObjectsInBox = obj.m_maxNumObjectsInBox ;
		m_userIdToObjId = obj.m_userIdToObjId;
		m_objects = obj.m_objects ;
		delete m_internalBoundingBox ; m_internalBoundingBox = 0;
		if (obj.m_internalBoundingBox) m_internalBoundingBox = new InternalBoundingBox(*obj.m_internalBoundingBox) ;
		return *this ;
	}

	const size_t getSizeInMemory() const
	{
		size_t res=0;
		res += sizeof(m_maxNumObjectsInBox);
		res += sizeof(m_extraExpansion);
		res += m_userIdToObjId.getSizeInMemory();
		res += sizeof(ObjectData) * m_objects.capacity();
		res += sizeof(m_internalBoundingBox);
		if(m_internalBoundingBox)
			res += m_internalBoundingBox->getSizeInMemory();

		return res;
	};

protected:
	int m_maxNumObjectsInBox ;
	double m_extraExpansion;
	SmallIntMap m_userIdToObjId;
	std::vector< ObjectData > m_objects;
	InternalBoundingBox *m_internalBoundingBox ;
};


#endif
