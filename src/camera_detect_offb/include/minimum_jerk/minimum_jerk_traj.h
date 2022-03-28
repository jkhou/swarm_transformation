#ifndef MINIMUM_JERK_TRAJ__H
#define MINIMUM_JERK_TRAJ__H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "matplotlibcpp.h"
#include "minimum_jerk/pva_table.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace plt = matplotlibcpp;

class Minimum_Jerk_Traj
{
public:
	Minimum_Jerk_Traj(std::string TABLE_PATH,double rate):delt_t(1.0/rate)
	{
		minimum_jerk_trajectory_time_table->csv2pva_table(TABLE_PATH);
	}
	Minimum_Jerk_Traj(){}
	~Minimum_Jerk_Traj(){}



	pva_table *minimum_jerk_trajectory_time_table = (pva_table*)malloc(sizeof(pva_table));

	double delt_t;
	double traj_point_num;

	std::vector<Eigen::Matrix<double ,9,1>> waypoint;
	std::vector<Eigen::Matrix<double ,9,1>> traj_point;

	std::vector<Eigen::Matrix<double ,9,1>> generate_minimum_jerk();
	void read_waypoint_csv(std::string CSV_PATH);
	void write_traj_point_csv(std::string CSV_PATH);
	void visual_traj_point();
	std::vector<Eigen::Matrix<double ,9,1>> generate_minimum_jerk_bisection(int swarm_ID, trajectory_msgs::JointTrajectoryPoint& t_init, trajectory_msgs::JointTrajectoryPoint& t_times);
	void init(std::string TABLE_PATH,double rate, double v_max_set,double a_max_set);
	bool check_dynamic_feasible(double T,double alpha,double beta,double gama,double a0,double v0,double p0);
	void set_waypoint(std::vector<Eigen::Matrix<double ,9,1>> waypoint_set);

	void solve_3_poly_root(double a,double b,double c,double d,std::vector<double> &X123);
	void solve_2_poly_root(double a,double b,double c,std::vector<double> &X123);

	double v_max;
	double a_max;

};

void Minimum_Jerk_Traj::init(std::string TABLE_PATH,double rate, double v_max_set,double a_max_set)
{
	minimum_jerk_trajectory_time_table->csv2pva_table(TABLE_PATH);
	delt_t=1.0/rate;
	v_max=v_max_set;
	a_max=a_max_set;
}

void Minimum_Jerk_Traj::set_waypoint(std::vector<Eigen::Matrix<double ,9,1>> waypoint_set)
{
	waypoint=waypoint_set;
}


void Minimum_Jerk_Traj::read_waypoint_csv(std::string CSV_PATH)
{
	waypoint.clear();
	std::ifstream _csvInput;
	_csvInput.open(CSV_PATH);


	std::string _Oneline;
	//读取一行数据
	while(getline(_csvInput, _Oneline))
	{
		std::istringstream _Readstr(_Oneline);	


		std::string _partOfstr;
		Eigen::Matrix<double ,9,1> single_waypoint;
		for(int i = 0; i < 9; i++)
		{	
			getline(_Readstr, _partOfstr, ',');
			single_waypoint(i) = atof(_partOfstr.c_str());	
		}
		std::cout<<"single_waypoint: "<<single_waypoint<<std::endl;
		waypoint.push_back(single_waypoint);
	}
	ROS_INFO("read_waypoint_csv SUCCESS");

}


std::vector<Eigen::Matrix<double ,9,1>> Minimum_Jerk_Traj::generate_minimum_jerk()
{
	traj_point.clear();
	for(int i=0;i<waypoint.size()-1;i++)
	{
		double T1, T2, T3, T;
		static double  last_T;
		double delt_x=waypoint[i+1](0)-waypoint[i](0);
		double delt_y=waypoint[i+1](1)-waypoint[i](1);
		double delt_z=waypoint[i+1](2)-waypoint[i](2);

		Eigen::Vector3d p0(waypoint[i](0),waypoint[i](1),waypoint[i](2));
		Eigen::Vector3d pf(waypoint[i+1](0),waypoint[i+1](1),waypoint[i+1](2));

		Eigen::Vector3d v0(waypoint[i](3),waypoint[i](4),waypoint[i](5));
		Eigen::Vector3d vf(waypoint[i+1](3),waypoint[i+1](4),waypoint[i+1](5));

		Eigen::Vector3d a0(waypoint[i](6),waypoint[i](7),waypoint[i](8));
		Eigen::Vector3d af(waypoint[i+1](6),waypoint[i+1](7),waypoint[i+1](8));

		T1 = minimum_jerk_trajectory_time_table->query_pva_table(delt_x,v0(0), vf(0), a0(0));
		T2 = minimum_jerk_trajectory_time_table->query_pva_table(delt_y, v0(1), vf(1), a0(1));
		T3 = minimum_jerk_trajectory_time_table->query_pva_table(delt_z, v0(2), vf(2), a0(2));

		if (T1 == -1 || T2 == -1 || T3 == -1) 
		{
			ROS_WARN("Can't find T");
		}
		else
		{
			T = T1 > T2 ? T1 : T2;
			T = T > T3 ? T : T3;
			T = T < 0.3 ? 0.3 : T;
		}

		int times = T / delt_t;

		Eigen::MatrixXd p,v,a,t;
		p = Eigen::MatrixXd::Zero(times, 3);
		v = Eigen::MatrixXd::Zero(times, 3);
		a = Eigen::MatrixXd::Zero(times, 3);
		t = Eigen::VectorXd::Zero(times);

		// calculate optimal jerk control points
		for(int ii=0; ii<3; ii++)
		{
			double delt_a = af(ii) - a0(ii);
			double delt_v = vf(ii) - v0(ii) - a0(ii)*T;
			double delt_p = pf(ii) - p0(ii) - v0(ii)*T - 0.5*a0(ii)*T*T;

			// % if vf is not free
			double alpha = delt_a*60/pow(T,3) - delt_v*360/pow(T,4) + delt_p*720/pow(T,5);
			double beta = -delt_a*24/pow(T,2) + delt_v*168/pow(T,3) - delt_p*360/pow(T,4);
			double gamma = delt_a*3/T - delt_v*24/pow(T,2) + delt_p*60/pow(T,3);

			for(int jj=0; jj<times; jj++)
			{
				double tt = (jj + 1)*delt_t;
				t(jj) = tt;
				p(jj,ii) = alpha/120*pow(tt,5) + beta/24*pow(tt,4) + gamma/6*pow(tt,3) + a0(ii)/2*pow(tt,2) + v0(ii)*tt + p0(ii);
				v(jj,ii) = alpha/24*pow(tt,4) + beta/6*pow(tt,3) + gamma/2*pow(tt,2) + a0(ii)*tt + v0(ii);
				a(jj,ii) = alpha/6*pow(tt,3) + beta/2*pow(tt,2) + gamma*tt + a0(ii);
			}
		}
		ROS_INFO("calculate one segment success");
		for(int jj=0;jj<times;jj++)
		{
			Eigen::Matrix<double ,9,1> single_traj_point;
			single_traj_point<<p(jj,0),p(jj,1),p(jj,2),v(jj,0),v(jj,1),v(jj,2),a(jj,0),a(jj,1),a(jj,2);
			// std::cout<<"single_traj_point: "<<single_traj_point<<std::endl;
			traj_point.push_back(single_traj_point);
		}

	}
	ROS_INFO("calculate traj_point success");
	return traj_point;
	// ROS_INFO("SSSSSSSSSSSSSSSS");

}

void Minimum_Jerk_Traj::solve_2_poly_root(double a,double b,double c,std::vector<double> &X123)
{
	double temp,root,r1,r2;
	temp = b * b - 4* a*c;
	root = sqrt(temp);
	if (temp < 0) {
	}
	else {
		if (b > 0) {
			r1 = 2 * c / (-b - root);
			r2 = (-b - root) / 2 / a;
		}
		else if(b<0){
			r1 = (-b + root) / 2 / a;
			r2 = 2 * c / (-b + root);
		}
		else {
			temp = c / a;
			r1 = sqrt(-temp);
			r2= -sqrt(-temp);	
		}
		X123.push_back(r1);
		X123.push_back(r2);

	}
}


void Minimum_Jerk_Traj::solve_3_poly_root(double a,double b,double c,double d,std::vector<double> &X123)
{
	/************************************************************************/
	/* 盛金公式求解三次方程的解 
	   德尔塔f=B^2-4AC
           这里只要了实根，虚根需要自己再整理下拿出来
	*/
	/************************************************************************/

	X123.clear();

	double A=b*b-3*a*c;
	double B=b*c-9*a*d;
	double C=c*c-3*b*d;
	double f=B*B-4*A*C;
	double i_value;
	double Y1,Y2;
	if (fabs(A)<1e-6 && fabs(B)<1e-6)//公式1
	{
		X123.push_back(-b/(3*a));
		X123.push_back(-b/(3*a));
		X123.push_back(-b/(3*a));
	}
	else if (fabs(f)<1e-6)   //公式3
	{
		double K=B/A;
		X123.push_back(-b/a+K);
		X123.push_back(-K/2);
		X123.push_back(-K/2);
	}
	else if (f>1e-6)      //公式2
	{
		Y1=A*b+3*a*(-B+sqrt(f))/2;
		Y2=A*b+3*a*(-B-sqrt(f))/2;
		double Y1_value=(Y1/fabs(Y1))*pow((double)fabs(Y1),1.0/3);
		double Y2_value=(Y2/fabs(Y2))*pow((double)fabs(Y2),1.0/3);
		X123.push_back((-b-Y1_value-Y2_value)/(3*a));//虚根我不要
		// //虚根还是看看吧，如果虚根的i小于0.1，则判定为方程的一根吧。。。
		// i_value=sqrt(3.0)/2*(Y1_value-Y2_value)/(3*a);
		// if (fabs(i_value)<1e-1)
		// {
		// 	X123.push_back((-b+0.5*(Y1_value+Y2_value))/(3*a));
		// }
	}
	else if (f<-1e-6)   //公式4
	{
		double T=(2*A*b-3*a*B)/(2*A*sqrt(A));
		double S=acos(T);
		X123.push_back((-b-2*sqrt(A)*cos(S/3))/(3*a));
		X123.push_back((-b+sqrt(A)*(cos(S/3)+sqrt(3.0)*sin(S/3)))/(3*a));
		X123.push_back((-b+sqrt(A)*(cos(S/3)-sqrt(3.0)*sin(S/3)))/(3*a));
	}
}



bool  Minimum_Jerk_Traj::check_dynamic_feasible(double T,double alpha,double beta,double gamma,double a0,double v0,double p0)
{
	// ROS_INFO("T %f,alpha %f,beta %f,gamma %f,a0 %f,v0 %f,p0 %f",T ,alpha ,beta ,gamma ,a0 ,v0 ,p0 );
	//check velocity
	std::vector<double> root;
	solve_3_poly_root(alpha/6.0,beta/2.0,gamma,a0,root);
	if(root.size()!=0)
	{
		for(int i=0;i<root.size();i++)
		{
			if(root[i]<T&&root[i]>0)
			{
				double tt=root[i];
				double v_local_max=(alpha/24*pow(tt,4) + beta/6*pow(tt,3) + gamma/2*pow(tt,2) + a0*tt + v0);
				// ROS_INFO("v_local_max: %f",v_local_max);
				if(fabs(v_local_max)>v_max)
					return false;
			}
		}
	}
	root.clear();
	solve_2_poly_root(alpha/2.0,beta,gamma,root);
	if(root.size()!=0)
	{
		for(int i=0;i<root.size();i++)
		{
			if(root[i]<T&&root[i]>0)
			{
				double tt=root[i];
				if(fabs(alpha/6*pow(tt,3) + beta/2*pow(tt,2) + gamma*tt + a0)>a_max)
					return false;
			}
		}
	}

	return true;
}


std::vector<Eigen::Matrix<double ,9,1>> Minimum_Jerk_Traj::generate_minimum_jerk_bisection(int swarm_ID, trajectory_msgs::JointTrajectoryPoint& t_init, trajectory_msgs::JointTrajectoryPoint& t_times)
{
	traj_point.clear();
	for(int i = 0; i < waypoint.size()-1;i++)
	{
		double T1, T2, T3;
		static double  last_T;
		double delt_x=waypoint[i+1](0)-waypoint[i](0);
		double delt_y=waypoint[i+1](1)-waypoint[i](1);
		double delt_z=waypoint[i+1](2)-waypoint[i](2);

		Eigen::Vector3d p0(waypoint[i](0),waypoint[i](1),waypoint[i](2));
		Eigen::Vector3d pf(waypoint[i+1](0),waypoint[i+1](1),waypoint[i+1](2));

		Eigen::Vector3d v0(waypoint[i](3),waypoint[i](4),waypoint[i](5));
		Eigen::Vector3d vf(waypoint[i+1](3),waypoint[i+1](4),waypoint[i+1](5));

		Eigen::Vector3d a0(waypoint[i](6),waypoint[i](7),waypoint[i](8));
		Eigen::Vector3d af(waypoint[i+1](6),waypoint[i+1](7),waypoint[i+1](8));
		// ROS_INFO("vmax %f amax %f ",v_max,a_max);

		if(fabs(delt_x) <= pow(v_max,2)/a_max)
		{
			T1=sqrt(4*fabs(delt_x)/a_max);
		}
		else
			T1=(fabs(delt_x)+pow(v_max,2)/a_max)/v_max;


		if(fabs(delt_y)<=pow(v_max,2)/a_max)
		{
			T2=sqrt(4*fabs(delt_y)/a_max);
		}
		else
			T2=(fabs(delt_y)+pow(v_max,2)/a_max)/v_max;

		if(fabs(delt_z)<=pow(v_max,2)/a_max)
		{
			T3=sqrt(4*fabs(delt_z)/a_max);
		}
		else
			T3=(fabs(delt_z)+pow(v_max,2)/a_max)/v_max;
		
		// ROS_INFO("T1 T2 T3 %f %f %f",T1,T2,T3);
		double T_initial = T1 > T2 ? T1 : T2;
        T_initial = T_initial> T3 ? T_initial : T3;

		// 如果是0号无人机，则计算每一段轨迹中的t_init
		if(swarm_ID == 0) {
            t_init.positions.push_back(T_initial);
        }
		else { // 如果是其他的无人机，则从0号无人机产生的数据中读取
            T_initial = t_init.positions[i];
		}

		double T_left = std::max(T_initial - 3.0, 0.1);
		double T_right = T_initial + 3.0;
		double T_middle = 0.5*(T_left + T_right);

		// 如果是0号无人机，则计算每一段轨迹中的t_init
//        if(swarm_ID == 0) {
//            t_init.positions.push_back(T_middle);
//        }
//        else { // 如果是其他的无人机，则从0号无人机产生的数据中读取
//            T_middle = t_init.positions[i];
//        }

//		ROS_INFO("swarm_ID: %d, T_initial %f T_left %f T_right %f T_middle %f",swarm_ID, T_initial,T_left,T_right,T_middle);
		while(T_right >= 0.2 + T_left)
		{
			bool flag=true;
			T_middle=0.5*(T_left+T_right);
//			ROS_INFO("T_middle: %f ////////////////////",T_middle);

			for(int ii=0;ii<3;ii++)
			{
				double delt_a = af(ii) - a0(ii);
				double delt_v = vf(ii) - v0(ii) - a0(ii)*T_middle;
				double delt_p = pf(ii) - p0(ii) - v0(ii)*T_middle - 0.5*a0(ii)*T_middle*T_middle;

				double alpha = delt_a*60/pow(T_middle,3) - delt_v*360/pow(T_middle,4) + delt_p*720/pow(T_middle,5);
				double beta = -delt_a*24/pow(T_middle,2) + delt_v*168/pow(T_middle,3) - delt_p*360/pow(T_middle,4);
				double gamma = delt_a*3/T_middle - delt_v*24/pow(T_middle,2) + delt_p*60/pow(T_middle,3);
			
				if(!check_dynamic_feasible(T_middle,alpha,beta,gamma,a0(ii),v0(ii),p0(ii)))
				{
					flag=false;
					break;
				}
			}

			// ROS_INFO("flag: %d",flag);
			if(flag)	
				T_right = T_middle;
			else
				T_left = T_middle;
		}


		int times = T_middle / delt_t;
//        ROS_INFO_THROTTLE(1, "swarm ID: %d, T_middle: %f, times: %d",swarm_ID, T_middle, times);
        std::cout <<  "i = " << i << "  times = " << times <<"  swarm_ID = " <<swarm_ID << "  T_initial = " << T_initial << "  T_middle = " << T_middle << std::endl;

        // 如果是0号无人机，则计算每一段轨迹中的t_init
        if(swarm_ID == 0) {
            t_times.positions.push_back(times);
        }
        else { // 如果是其他的无人机，则从0号无人机产生的数据中读取
            times = t_times.positions[i];
        }

		Eigen::MatrixXd p,v,a,t;
		p = Eigen::MatrixXd::Zero(times, 3);
		v = Eigen::MatrixXd::Zero(times, 3);
		a = Eigen::MatrixXd::Zero(times, 3);
		t = Eigen::VectorXd::Zero(times);

		// calculate optimal jerk control points
		for(int ii=0; ii<3; ii++)
		{
			double delt_a = af(ii) - a0(ii);
			double delt_v = vf(ii) - v0(ii) - a0(ii)*T_middle;
			double delt_p = pf(ii) - p0(ii) - v0(ii)*T_middle - 0.5*a0(ii)*T_middle*T_middle;

			// % if vf is not free
			double alpha = delt_a*60/pow(T_middle,3) - delt_v*360/pow(T_middle,4) + delt_p*720/pow(T_middle,5);
			double beta = -delt_a*24/pow(T_middle,2) + delt_v*168/pow(T_middle,3) - delt_p*360/pow(T_middle,4);
			double gamma = delt_a*3/T_middle - delt_v*24/pow(T_middle,2) + delt_p*60/pow(T_middle,3);

			for(int jj=0; jj<times; jj++)
			{
				double tt = (jj + 1)*delt_t;
				t(jj) = tt;
				p(jj,ii) = alpha/120*pow(tt,5) + beta/24*pow(tt,4) + gamma/6*pow(tt,3) + a0(ii)/2*pow(tt,2) + v0(ii)*tt + p0(ii);
				v(jj,ii) = alpha/24*pow(tt,4) + beta/6*pow(tt,3) + gamma/2*pow(tt,2) + a0(ii)*tt + v0(ii);
				a(jj,ii) = alpha/6*pow(tt,3) + beta/2*pow(tt,2) + gamma*tt + a0(ii);
			}
		}
		ROS_INFO_THROTTLE(1, "calculate one segment success ==========");
		for(int jj=0;jj < times;jj++)
		{
			Eigen::Matrix<double ,9,1> single_traj_point;
			single_traj_point<<p(jj,0),p(jj,1),p(jj,2),v(jj,0),v(jj,1),v(jj,2),a(jj,0),a(jj,1),a(jj,2);
			// std::cout<<"single_traj_point: "<<single_traj_point<<std::endl;
			traj_point.push_back(single_traj_point);
		}

	}
	ROS_INFO_THROTTLE(1, "calculate traj_point success");
	return traj_point;
	// ROS_INFO("SSSSSSSSSSSSSSSS");

}





void Minimum_Jerk_Traj::write_traj_point_csv(std::string CSV_PATH)
{
	
	std::ofstream outFile;
	outFile.open(CSV_PATH); // 打开模式可省略

	std::vector<Eigen::Matrix<double ,9,1>>::iterator it = traj_point.begin();

	ROS_INFO("total traj cost %f s",traj_point.size()*delt_t);
	for(; it != traj_point.end(); ++it)
	{
		outFile << (*it)(0) << ',' << (*it)(1) << ',' << (*it)(2) <<',' << (*it)(3)<<',' << (*it)(4)<<',' << (*it)(5)
		<<',' << (*it)(6)<<',' << (*it)(7)<<',' << (*it)(8)<<std::endl;
	}

	outFile.close();

}

void Minimum_Jerk_Traj::visual_traj_point()
{

    std::vector<std::vector<double>> plan_point(traj_point.size());
	std::vector<std::vector<double>> way_point(waypoint.size());

    for(int i=0;i<traj_point.size();i++)
    {
        plan_point[0].push_back(-traj_point[i](1));
        plan_point[1].push_back(traj_point[i](0));
        plan_point[2].push_back(traj_point[i](2));
    }

	for(int i=0;i<way_point.size();i++)
    {
        way_point[0].push_back(-waypoint[i](1));
        way_point[1].push_back(waypoint[i](0));
        way_point[2].push_back(waypoint[i](2));
    }
	plt::plot(plan_point[0],plan_point[1]);
	plt::plot(way_point[0],way_point[1]);
	

    plt::grid(true);
    plt::show();
	plt::axis("square");
}



#endif