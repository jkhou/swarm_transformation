#ifndef PVA_TABLE__H
#define PVA_TABLE__H


#include <ros/ros.h> 
#include <serial/serial.h>   
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <string>
#include <string.h>
#include <sstream>
#include <iostream>
#include <fstream> 
#include <vector>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <algorithm>


typedef struct pva_table pva_table;
struct pva_table {
    int dim_num;
    int *dim_size;
    uint64_t *dim_interval;
    uint64_t table_size;
    double *table;
    double rez;
    double *pva_limit;

    void construct_pva_table(int dim1_size, int dim2_size, int dim3_size, int dim4_size, double resolution) {
        this->dim_num = 4;
        this->dim_size = (int*)malloc(sizeof(int)*this->dim_num);
        this->dim_interval = (uint64_t*)malloc(sizeof(uint64_t)*this->dim_num);

        this->dim_size[0] = dim1_size;
        this->dim_size[1] = dim2_size;
        this->dim_size[2] = dim3_size;
        this->dim_size[3] = dim4_size;

        this->dim_interval[3] = 1;
        this->dim_interval[2] = dim_interval[3] * dim4_size;
        this->dim_interval[1] = dim_interval[2] * dim3_size;
        this->dim_interval[0] = dim_interval[1] * dim2_size;

        this->table_size = this->dim_interval[0] * dim1_size;
        this->table = (double*)malloc(sizeof(double)*this->table_size);

        this->rez = resolution;

        this->pva_limit = (double*)malloc(sizeof(double)*3);
        this->pva_limit[0] = this->rez*double(dim1_size/2);
        this->pva_limit[1] = this->rez*double(dim2_size/2);
        this->pva_limit[2] = this->rez*double(dim4_size/2);
    }

    void compute_idx_from_pva(double dlt_p, double v0, double vf, double a0,
                              int &idx1, int &idx2, int &idx3, int &idx4) {
        idx1 = round(dlt_p/this->rez) + this->dim_size[0]/2;
        idx2 = round(v0/this->rez) + this->dim_size[1]/2;
        idx3 = round(vf/this->rez) + this->dim_size[2]/2;
        idx4 = round(a0/this->rez) + this->dim_size[3]/2;
    }

    double query_pva_table(double dlt_p, double v0, double vf, double a0) {
        double coefficient=1;

        if (fabs(dlt_p) > this->pva_limit[0]) 
        {        
            static_assert("Invalid input!", "");
            
            coefficient=fabs(dlt_p)/this->pva_limit[0];

            dlt_p=this->pva_limit[0]*dlt_p/fabs(dlt_p);
        }
        if (fabs(v0) > this->pva_limit[1]) static_assert("Invalid input!", "");
        if (fabs(vf) > this->pva_limit[1]) static_assert("Invalid input!", "");
        if (fabs(a0) > this->pva_limit[2]) static_assert("Invalid input!", "");

        int idx1, idx2, idx3, idx4;
        this->compute_idx_from_pva(dlt_p, v0, vf, a0, idx1, idx2, idx3, idx4);

        uint64_t idx = idx1*this->dim_interval[0] + idx2*this->dim_interval[1] +
                       idx3*this->dim_interval[2] + idx4*this->dim_interval[3];

        return this->table[idx]*(coefficient);
    }

    void csv2pva_table(const std::string &str) {
        int tmp_dim_size[4];
        double tmp_rez;

        std::ifstream infile(str, std::ios::in);
        std::string tmp_str;

        for (int i = 0; i < 4; ++i) {
            getline(infile, tmp_str, ',');
            tmp_dim_size[i] = std::stoi(tmp_str);
        }

        getline(infile, tmp_str);
        tmp_rez = std::stod(tmp_str);

        this->construct_pva_table(tmp_dim_size[0], tmp_dim_size[1],
                                  tmp_dim_size[2], tmp_dim_size[3], tmp_rez);

        for (uint64_t i = 0; i < this->table_size; ++i) {
            getline(infile, tmp_str, ',');
            this->table[i] = std::stod(tmp_str);
        }
    }
};



#endif