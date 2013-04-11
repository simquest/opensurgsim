void getCollisions(const AABBTree* other, 
	std::vector<int>& thisIdList, std::vector<int>& otherIdList) const
{
	if (m_internalBoundingBox == 0 || other->m_internalBoundingBox == 0) return ;
	double tolerance = m_extraExpansion + other->m_extraExpansion;

	std::vector<std::pair<InternalBoundingBox*,InternalBoundingBox*> > boxes_to_test;
	if (m_internalBoundingBox->intersect(*(other->m_internalBoundingBox), tolerance))
	{
		boxes_to_test.push_back(std::make_pair(m_internalBoundingBox, other->m_internalBoundingBox));
	}
	while (! boxes_to_test.empty())
	{
		InternalBoundingBox* currThisBox  = boxes_to_test.back().first;
		InternalBoundingBox* currOtherBox = boxes_to_test.back().second;
		boxes_to_test.pop_back();

		if (currThisBox->m_obj_ids.empty() && currOtherBox->m_obj_ids.empty())
		{
			if (currThisBox->m_child1)
			{
				if (currOtherBox->m_child1 && currThisBox->m_child1->intersect(*(currOtherBox->m_child1), tolerance))
				{
					boxes_to_test.push_back(std::make_pair(currThisBox->m_child1, currOtherBox->m_child1));
				}
				if (currOtherBox->m_child2 && currThisBox->m_child1->intersect(*(currOtherBox->m_child2), tolerance))
				{
					boxes_to_test.push_back(std::make_pair(currThisBox->m_child1, currOtherBox->m_child2));
				}
			}
			if (currThisBox->m_child2)
			{
				if (currOtherBox->m_child1 && currThisBox->m_child2->intersect(*(currOtherBox->m_child1), tolerance))
				{
					boxes_to_test.push_back(std::make_pair(currThisBox->m_child2, currOtherBox->m_child1));
				}
				if (currOtherBox->m_child2 && currThisBox->m_child2->intersect(*(currOtherBox->m_child2), tolerance))
				{
					boxes_to_test.push_back(std::make_pair(currThisBox->m_child2, currOtherBox->m_child2));
				}
			}
		}
		else if (currThisBox->m_obj_ids.empty())
		{
			if (currThisBox->m_child1 && currThisBox->m_child1->intersect(*currOtherBox, tolerance))
			{
				boxes_to_test.push_back(std::make_pair(currThisBox->m_child1, currOtherBox));
			}
			if (currThisBox->m_child2 && currThisBox->m_child2->intersect(*currOtherBox, tolerance))
			{
				boxes_to_test.push_back(std::make_pair(currThisBox->m_child2, currOtherBox));
			}
		}
		else if (currOtherBox->m_obj_ids.empty())
		{
			if (currOtherBox->m_child1 && currThisBox->intersect(*(currOtherBox->m_child1), tolerance))
			{
				boxes_to_test.push_back(std::make_pair(currThisBox, currOtherBox->m_child1));
			}
			if (currOtherBox->m_child2 && currThisBox->intersect(*(currOtherBox->m_child2), tolerance))
			{
				boxes_to_test.push_back(std::make_pair(currThisBox, currOtherBox->m_child2));
			}
		}
		else
		{
			bool checkAgain = currThisBox->m_obj_ids.size() > 1 || currOtherBox->m_obj_ids.size() > 1;
			std::vector<int>::iterator itr1, itr2;
			for( itr1 = currThisBox->m_obj_ids.begin(); itr1 !=currThisBox->m_obj_ids.end(); ++itr1 )
			{
				const ObjectData& obj1 = m_objects[*itr1];
				for( itr2 = currOtherBox->m_obj_ids.begin(); itr2 !=currOtherBox->m_obj_ids.end(); ++itr2 )
				{
					const ObjectData& obj2 = other->m_objects[*itr2];
					if (checkAgain && !obj1.m_aabb.broadPhase_isIntersecting(obj2.m_aabb, tolerance))
						continue;
					thisIdList.push_back(obj1.m_userId);
					otherIdList.push_back(obj2.m_userId);
				}
			}
		}
	}
}

void getSelfCollisions(std::vector<int>& idList1, std::vector<int>& idList2) const
{
	if (m_internalBoundingBox == 0) return ;

	double tolerance = m_extraExpansion * 2.0;

	std::vector<std::pair<InternalBoundingBox*,InternalBoundingBox*> > boxes_to_test;
	boxes_to_test.push_back(std::make_pair(m_internalBoundingBox, m_internalBoundingBox));
	while (! boxes_to_test.empty())
	{
		InternalBoundingBox* currThisBox  = boxes_to_test.back().first;
		InternalBoundingBox* currOtherBox = boxes_to_test.back().second;
		boxes_to_test.pop_back();

		if (currThisBox->m_obj_ids.empty() && currOtherBox->m_obj_ids.empty())
		{
			if (currThisBox->m_child1)
			{
				if (currOtherBox->m_child1 && currThisBox->m_child1->intersect(*(currOtherBox->m_child1), tolerance))
				{
					boxes_to_test.push_back(std::make_pair(currThisBox->m_child1, currOtherBox->m_child1));
				}
				if (currOtherBox->m_child2 && currThisBox->m_child1->intersect(*(currOtherBox->m_child2), tolerance))
				{
					boxes_to_test.push_back(std::make_pair(currThisBox->m_child1, currOtherBox->m_child2));
				}
			}
			if (currThisBox->m_child2)
			{
				if (currOtherBox->m_child1 && currThisBox->m_child2->intersect(*(currOtherBox->m_child1), tolerance))
				{
					boxes_to_test.push_back(std::make_pair(currThisBox->m_child2, currOtherBox->m_child1));
				}
				if (currOtherBox->m_child2 && currThisBox->m_child2->intersect(*(currOtherBox->m_child2), tolerance))
				{
					boxes_to_test.push_back(std::make_pair(currThisBox->m_child2, currOtherBox->m_child2));
				}
			}
		}
		else if (currThisBox->m_obj_ids.empty())
		{
			if (currThisBox->m_child1 && currThisBox->m_child1->intersect(*currOtherBox, tolerance))
			{
				boxes_to_test.push_back(std::make_pair(currThisBox->m_child1, currOtherBox));
			}
			if (currThisBox->m_child2 && currThisBox->m_child2->intersect(*currOtherBox, tolerance))
			{
				boxes_to_test.push_back(std::make_pair(currThisBox->m_child2, currOtherBox));
			}
		}
		else if (currOtherBox->m_obj_ids.empty())
		{
			if (currOtherBox->m_child1 && currThisBox->intersect(*(currOtherBox->m_child1), tolerance))
			{
				boxes_to_test.push_back(std::make_pair(currThisBox, currOtherBox->m_child1));
			}
			if (currOtherBox->m_child2 && currThisBox->intersect(*(currOtherBox->m_child2), tolerance))
			{
				boxes_to_test.push_back(std::make_pair(currThisBox, currOtherBox->m_child2));
			}
		}
		else
		{
			bool checkAgain = currThisBox->m_obj_ids.size() > 1 || currOtherBox->m_obj_ids.size() > 1;
			std::vector<int>::iterator itr1, itr2;
			for( itr1 = currThisBox->m_obj_ids.begin(); itr1 !=currThisBox->m_obj_ids.end(); ++itr1 )
			{
				const ObjectData& obj1 = m_objects[*itr1];
				for( itr2 = currOtherBox->m_obj_ids.begin(); itr2 !=currOtherBox->m_obj_ids.end(); ++itr2 )
				{
					if (*itr2 <= *itr1) continue; //! Avoid self-triangle collision and duplicates

					const ObjectData& obj2 = m_objects[*itr2];
					if (checkAgain && !obj1.m_aabb.broadPhase_isIntersecting(obj2.m_aabb, tolerance))
						continue;
					idList1.push_back(obj1.m_userId);
					idList2.push_back(obj2.m_userId);
				}
			}
		}
	}

	//std::vector<int> tmpList1, tmpList2;
	//this->getCollisions(this, tmpList1, tmpList2);

	//bool duplicated;
	//for (int ii = 0; ii < (int)tmpList1.size(); ++ii)
	//{
	//	if (tmpList1[ii] == tmpList2[ii]) continue;

	//	duplicated = false;
	//	for (int jj = 0; jj < (int)idList1.size(); ++jj)
	//	{
	//		if (tmpList1[ii] == idList1[jj] && tmpList2[ii] == idList2[jj]) duplicated = true;
	//		if (tmpList1[ii] == idList2[jj] && tmpList2[ii] == idList1[jj]) duplicated = true;
	//	}
	//	if (duplicated) continue;

	//	if (tmpList1[ii] > tmpList2[ii])
	//	{
	//		idList1.push_back(tmpList2[ii]);
	//		idList2.push_back(tmpList1[ii]);
	//	}
	//	else
	//	{
	//		idList1.push_back(tmpList1[ii]);
	//		idList2.push_back(tmpList2[ii]);
	//	}
	//}
}

void getCollisions(const _AABB_& aabb, std::vector<int>& idList) const
{
	if( m_internalBoundingBox == NULL ) return ;
	std::vector<InternalBoundingBox *> bounding_box_to_test ;
	bounding_box_to_test.push_back(m_internalBoundingBox) ;
	InternalBoundingBox * curr_box ;
	while( !bounding_box_to_test.empty())
	{
		curr_box = bounding_box_to_test.back() ;
		bounding_box_to_test.pop_back() ;

		if( !curr_box->intersect(aabb, m_extraExpansion) ) continue ;

		if( curr_box->m_obj_ids.empty() )
		{
			if( curr_box->m_child1 )
				bounding_box_to_test.push_back(curr_box->m_child1);
			if( curr_box->m_child2 )
				bounding_box_to_test.push_back(curr_box->m_child2);
		}
		else
		{
			bool checkAgain = curr_box->m_obj_ids.size() > 1;
			std::vector<int>::iterator itr = curr_box->m_obj_ids.begin() ;
			for( ; itr !=curr_box->m_obj_ids.end(); ++itr )
			{
				if (checkAgain && !m_objects[*itr].m_aabb.broadPhase_isIntersecting(aabb, m_extraExpansion))
					continue;
				idList.push_back(m_objects[(*itr)].m_userId);
			}
		}
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