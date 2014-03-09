#include "Filtry.h"

// AntyNaN
pcl::PointCloud<pcl::PointXYZ>::Ptr AntyNaNFilter(
		pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr Output(
			new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough < pcl::PointXYZ > pass;
	pass.setInputCloud(Cloud);
	pass.filter(*Output);

	return Output;
}

// VoxelGrid
pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelGridFilter(
		pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float LeafSize) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr Output(
			new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid < pcl::PointXYZ > Voxel;
	Voxel.setInputCloud(Cloud);
	Voxel.setLeafSize(LeafSize, LeafSize, LeafSize);
	Voxel.filter(*Output);

	return Output;
}

// AntySzum
pcl::PointCloud<pcl::PointXYZ>::Ptr AntySzumFilter(
		pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, float Mean, float Stddev) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr Output(
			new pcl::PointCloud<pcl::PointXYZ>);

	pcl::StatisticalOutlierRemoval < pcl::PointXYZ > sor;
	sor.setInputCloud(Cloud);
	sor.setMeanK(Mean);
	sor.setStddevMulThresh(Stddev);
	sor.filter(*Output);

	return Output;
}

/// KOLOR
// AntyNaN
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr AntyNaNFilter(
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Output(
			new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::PassThrough < pcl::PointXYZRGBA > pass;
	pass.setInputCloud(Cloud);
	pass.filter(*Output);

	return Output;
}

// VoxelGrid
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr VoxelGridFilter(
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud, float LeafSize) {

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Output(
			new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::VoxelGrid < pcl::PointXYZRGBA > Voxel;
	Voxel.setInputCloud(Cloud);
	Voxel.setLeafSize(LeafSize, LeafSize, LeafSize);
	Voxel.filter(*Output);

	return Output;
}

