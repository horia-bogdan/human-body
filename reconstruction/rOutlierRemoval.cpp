#include "rOutlierRemoval.h"

#include <pcl/search/kdtree.h>
#include <list>

namespace rec3D

{

rOutlierRemoval::rOutlierRemoval(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
	:input(point_cloud),
	epsilon(0.0175),
	min_points(50)
{
}

void rOutlierRemoval::apply(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output)
{
	int size = input->size();
	
	labels.resize(size, -2);

	pcl::search::KdTree<pcl::PointXYZRGB> tree;
	tree.setInputCloud(input);

	int c = 0;

	for (int i = 0; i < size; i++)
	{
		if (labels[i] != -2)
		{
			continue;
		}

		std::vector<int> indices;
		std::vector<float> sq_distances;
		tree.radiusSearch(
			input->operator[](i), 
			epsilon, 
			indices, 
			sq_distances);

		if (indices.size() < min_points)
		{
			labels[i] = -1;
			continue;
		}

		labels[i] = c;
		std::sort(indices.begin(), indices.end());
		std::set<int> indices_set(indices.begin(), indices.end());

		std::list<std::set<int>::iterator> order_list;
		for (std::set<int>::iterator it = indices_set.begin(); 
			it != indices_set.end(); 
			++it)
		{
			order_list.push_back(it);
		}

		for(std::list<std::set<int>::iterator>::iterator it = order_list.begin(); 
			it != order_list.end(); 
			++it)
		{
			if (labels[**it] == -1)
			{
				labels[**it] = c;
			}
			if (labels[**it] != -2)
			{
				continue;
			}

			labels[**it] = c;
			std::vector<int> indices2;
			std::vector<float> sq_distances2;

			tree.radiusSearch(
				input->operator[](**it), 
				epsilon, 
				indices2, 
				sq_distances2);

			if (indices2.size() >= min_points)
			{
				std::sort(indices2.begin(), indices2.end());

				for (std::vector<int>::iterator i2_it = indices2.begin();
					i2_it != indices2.end();
					++i2_it)
				{
					std::pair<std::set<int>::iterator, bool> return_value;

					return_value = indices_set.insert(*i2_it);

					if (return_value.second == true)
					{
						order_list.push_back(return_value.first);
					}
				}
			}
		}
		++c;
	}

	std::vector<int> cluster_size;
	cluster_size.resize(c, 0);
	int max_size_label = 0;
	
	for (int i = 0; i < size; i++)
	{
		if (labels[i] < 0)
		{
			continue;
		}
		cluster_size[labels[i]]++;
		
		if (cluster_size[labels[i]] > cluster_size[max_size_label])
		{
			max_size_label = labels[i];
		}
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(
		new pcl::PointCloud<pcl::PointXYZRGB>());
	for (int i = 0; i < size; i++)
	{
		if (labels[i] == max_size_label)
		{
			out->push_back(input->operator[](i));
		}
	}

	output = out;
}

rOutlierRemoval::~rOutlierRemoval()
{
}

}
