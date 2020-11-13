//
// Created by firas on 27.7.2020.
//

#include "Product.h"
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>

using namespace std;

Product::Product(const string& barcode, bool with_color, const string& folder_path)
{
	vector<cv::Point3f> tmp_3d_map;
	vector<cv::Scalar> tmp_colors;
	string x, y, z, r, g, b;
	ifstream point3dfile, colorfile;

	cout << "Reading:" << endl;
	cv::FileStorage descriptorsfile;
	descriptorsfile.open(folder_path + barcode + "_descriptors" + ".xml", cv::FileStorage::READ);
	try
	{
		if (!descriptorsfile.isOpened())
		{
			throw 15;
		}
		cout << "desc File is opened\n";
	}
	catch (int err)
	{
		cout << "wrong descriptors File !!!! \n" << err;

	}
	cv::Mat pose2;
	point3dfile.open(folder_path + barcode + "_points" + ".csv");
	if (with_color)
		colorfile.open(folder_path + barcode + "_colors" + ".csv");
	else
		colorfile.open(folder_path + barcode + "_points" + ".csv");

	string mapERROR = "wrong 3d map file\n";
	try
	{
		if (!point3dfile.is_open())
		{
			throw mapERROR;
		}
		cout << " 3d-P File is opened\n";

	}
	catch (string mapERROR)
	{
		cout << mapERROR;
	}

	int i = 0;
	int counter = 0;
	cv::Mat tmp_descriptors;
	while (point3dfile.good())
	{
		switch (counter)
		{
		case 0:
			getline(point3dfile, x, ',');
			getline(colorfile, r, ',');
			counter++;

			break;
		case 1:
			getline(point3dfile, y, ',');
			getline(colorfile, g, ',');
			counter++;
			break;
		case 2:
			string st = "desc" + to_string(i);
			descriptorsfile[st] >> pose2;
			getline(point3dfile, z, '\n');
			getline(colorfile, b, '\n');
			float xF = atof(x.c_str());
			float yF = atof(y.c_str());
			float zF = atof(z.c_str());
			float rF = atof(r.c_str());
			float gF = atof(g.c_str());
			float bF = atof(b.c_str());
			if (rF > 245 && gF > 245 && bF > 245)
			{ continue; }
			tmp_descriptors.push_back({ pose2 });
			tmp_3d_map.emplace_back(xF, yF, zF);
			tmp_colors.emplace_back(cv::Scalar(rF, gF, bF));
			counter = 0;
			i++;
			break;
		}
	}

	products_descriptors_ = tmp_descriptors;
	products_3d_map_ = tmp_3d_map;
	products_colors_ = tmp_colors;
	barcode_ = barcode;
	descriptorsfile.release();
	point3dfile.close();
	colorfile.close();
}
