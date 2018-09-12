#include <cstdlib>
#include <memory>
#include <cmath>
#include <vector>
#include <set>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include<vector>
#include<deque>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class Augmenter : public RFModule
{
    vector<Vector> all_points;
    vector<vector<unsigned char>> all_colors;
    vector<Vector> new_points;
    vector<Matrix> desired_pc_poses;
    vector<Matrix> final_pc_poses;


    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string file=rf.find("file").asString();
        string homeContextPath=rf.getHomeContextPath().c_str();
        string object=rf.find("object").asString();
        ifstream fin(file.c_str());
        if (!fin.is_open())
        {
            yError()<<"Unable to open file \""<<file<<"\"";
            return false;
        }

        Vector p(3);
        vector<unsigned int> c_(3);
        vector<unsigned char> c(3);

        string line;
        while (getline(fin,line))
        {
            istringstream iss(line);
            if (!(iss>>p[0]>>p[1]>>p[2]))
                break;
            all_points.push_back(p);

            fill(c_.begin(),c_.end(),120);
            iss>>c_[0]>>c_[1]>>c_[2];
            c[0]=(unsigned char)c_[0];
            c[1]=(unsigned char)c_[1];
            c[2]=(unsigned char)c_[2];
            all_colors.push_back(c);
        }

        Vector center(3,0.0);

        double height_plane=-0.18;

        double x_rotation;
        double y_rotation;
        double z_rotation;

        double theta_zyz, phi_zyz, psi_zyz;
        theta_zyz=phi_zyz=psi_zyz=0.0;

        for (size_t j=0; j<24; j++)
        {
            Matrix pose(4,4);
            pose.eye();

            center[0]=-0.4;

            if (j%3==0)
                center[1]= -0.1;
            else if ((j-1)%3==0)
                center[1]=0.0;
            else
                center[1]=0.1;

            // Center 2 need to be defined later, after rotation

            x_rotation=M_PI/10.0;
            y_rotation=-M_PI/10.0;

            if (j<3)
                z_rotation=-M_PI/5.0;
            else if (j>=3 && j<6)
                z_rotation=0.0;
            else if (j>=6 && j<9)
                z_rotation=M_PI/5.0;
            else if (j>=9 && j<12)
                z_rotation=M_PI/3.0;

            else if (j>=12 && j<15)
            {
                x_rotation=M_PI/2 + M_PI/10.0;
                z_rotation=-M_PI/5.0;
                y_rotation=0.0;
            }
            else if (j>=15 && j<18)
            {
                x_rotation=M_PI/2 + M_PI/10.0;
                z_rotation=0.0;
                y_rotation=0.0;
            }
            else if (j>=18 && j<21)
            {
                x_rotation=M_PI/2 + M_PI/10.0;
                z_rotation=-M_PI/4.0;
                y_rotation=0.0;
            }
            else if (j>=21 && j<24)
            {
                x_rotation=M_PI/2 + M_PI/10.0;
                z_rotation=M_PI/5.0;
                y_rotation=0.0;
            }

            // Test only traslation
            x_rotation=0.0;
            y_rotation=0.0;
            z_rotation=0.0;

            /*phi_zyz=atan2(-sin(x_rotation)*cos(z_rotation),sin(z_rotation)*sin(x_rotation));
            theta_zyz=atan2(sin(x_rotation), cos(x_rotation));
            psi_zyz=atan2(sin(x_rotation), 0);*/

            double first_term=sin(z_rotation)*sin(y_rotation) - sin(x_rotation)*cos(z_rotation) * cos(y_rotation);
            double second_term=cos(z_rotation)*sin(y_rotation)+sin(z_rotation)*sin(x_rotation)*cos(y_rotation);

            phi_zyz=atan2(first_term,
                          second_term);

            theta_zyz=atan2(sqrt(first_term * first_term + second_term * second_term) , cos(x_rotation)*cos(y_rotation));
            psi_zyz=atan2(sin(x_rotation), sin(y_rotation) * sin(x_rotation));

            Vector eulers(3,0.0);
            eulers[0]=phi_zyz; eulers[1]=theta_zyz; eulers[2]=psi_zyz;

            pose.setSubmatrix(euler2dcm(eulers), 0,0);
            pose.setSubcol(center, 0,3);

            desired_pc_poses.push_back(pose);
        }

        Vector x0(3,0.0);
        Matrix R_pc(4,4);
        R_pc.eye();
        R_pc.setSubmatrix(computeInitialOrientation(all_points, x0), 0, 0);

        Matrix bounding_box(3,2);
        bounding_box=computeBoundingBox(all_points, R_pc);

        //yDebug()<<"bb "<<bounding_box.toString();


        bounding_box = R_pc.submatrix(0,2,0,2) * bounding_box;

        yDebug()<<"bb "<<bounding_box.toString();
        x0[0] = (bounding_box(0,0)+bounding_box(0,1))/2;
        x0[1] = (bounding_box(1,0)+bounding_box(1,1))/2;
        x0[2] = (bounding_box(2,0)+bounding_box(2,1))/2;

        R_pc.setSubcol(x0,0,3);

        Vector dim(3,0.0);

        dim[0]=(-bounding_box(0,0)+bounding_box(0,1))/2;
        dim[1]=(-bounding_box(1,0)+bounding_box(1,1))/2;
        dim[2]=(-bounding_box(2,0)+bounding_box(2,1))/2;



        // At the beginning let's leave the current z coordinate
        for (size_t j=0; j<24; j++)
        {
            desired_pc_poses[j](2,3)=x0[2];
        }

        for (size_t j=0; j<24; j++)
        {

             new_points.clear();
             Matrix transformation(4,4);
             transformation.eye();



             //transformation=desired_pc_poses[j]*SE3inv(R_pc);
             // Test only trasl
             transformation=desired_pc_poses[j];


             for (size_t i=0; i<all_points.size();i++)
             {
                 Vector point_rot(4,1.0);
                 point_rot.setSubvector(0,all_points[i]);
                 point_rot= transformation * point_rot;

                 new_points.push_back(point_rot.subVector(0,2));
             }


             Matrix bounding_box(3,2);
             bounding_box=computeBoundingBox(new_points, desired_pc_poses[j]);

             Vector x0(3,0.0);

             bounding_box = desired_pc_poses[j].submatrix(0,2,0,2) * bounding_box;
             x0[0] = (bounding_box(0,0)+bounding_box(0,1))/2;
             x0[1] = (bounding_box(1,0)+bounding_box(1,1))/2;
             x0[2] = (bounding_box(2,0)+bounding_box(2,1))/2;


             Vector dim_4(4,1.0);
             dim_4.setSubvector(0,dim);

             Vector rotated_dim(3,0.0);
             rotated_dim=(transformation*dim_4).subVector(0,2);

             if (x0[2] != height_plane + rotated_dim[2])
             {
                 Matrix extra_trasl(4,4);
                 extra_trasl.eye();
                 double diff = x0[2] - (height_plane +rotated_dim[2]);
                 extra_trasl.setSubcol(x0[2] - diff, 0, 3);


                 vector<Vector> aux_points;
                 aux_points=new_points;

                 new_points.clear();

                 for (size_t i=0; i<aux_points.size();i++)
                 {
                     Vector point_rot(4,1.0);
                     point_rot.setSubvector(0,aux_points[i]);
                     point_rot= extra_trasl * point_rot;

                     new_points.push_back(point_rot.subVector(0,2));
                 }


             }

            ofstream fout_off, fout;
            stringstream ss;
            ss << j;

            string count_file_str=ss.str();
            fout_off.open(("point_cloud_"+object+"+rotation_no_"+count_file_str+".off").c_str());
            fout.open(("point_cloud_"+object+"+rotation_no_"+count_file_str).c_str());

            if (fout.is_open() && fout_off.is_open())
            {
                fout_off<<"COFF"<<endl;
                fout_off<<new_points.size()<<" 0 0"<<endl;
                fout_off<<endl;


                for (size_t i=0; i<new_points.size(); i++)
                {
                    int r=all_colors[i][0];
                    int g=all_colors[i][1];
                    int b=all_colors[i][2];

                    fout<<new_points[i].toString(3,3)<<" "<<r<<" "<<g<<" "<<b<<endl;

                    fout_off<<new_points[i].toString(3,3)<<" "<<r<<" "<<g<<" "<<b<<endl;
                }

                fout<<endl;
                fout_off<<endl;
            }
            else
                yError()<<" Some problems in opening output file!";

            fout.close();
            fout_off.close();
        }
    }

    /****************************************************************/
    Matrix computeBoundingBox(vector<Vector> &points, const Matrix &T_cost)
    {
        Matrix BB(3,2);

        BB(0,0)=numeric_limits<double>::infinity();
        BB(1,0)=numeric_limits<double>::infinity();
        BB(2,0)=numeric_limits<double>::infinity();
        BB(0,1)=-numeric_limits<double>::infinity();
        BB(1,1)=-numeric_limits<double>::infinity();
        BB(2,1)=-numeric_limits<double>::infinity();

        Matrix T = SE3inv(T_cost);

        for (size_t i=0; i<points.size();i++)
        {
            Vector point(4,1.0);
            point.setSubvector(0,points[i]);
            point = T * point;
            //point=points[i];
            if(BB(0,0)>point[0])
               BB(0,0)=point[0];

            if(BB(0,1)<point[0])
                BB(0,1)=point[0];

            if(BB(1,0)>point[1])
                BB(1,0)=point[1];

            if(BB(1,1)<point[1])
                BB(1,1)=point[1];

            if(BB(2,0)>point[2])
                BB(2,0)=point[2];

            if(BB(2,1)<point[2])
                BB(2,1)=point[2];
        }

        return BB;
    }


    /****************************************************************/
    Matrix computeInitialOrientation(vector<Vector> &point_cloud, Vector &x0)
    {
        Matrix M=zeros(3,3);
        Matrix R(3,3);
        Matrix u(3,3);
        Matrix v(3,3);

        Vector s(3,0.0);
        Vector n(3,0.0);
        Vector o(3,0.0);
        Vector a(3,0.0);

        for (size_t i=0;i<point_cloud.size(); i++)
        {
            Vector &point=point_cloud[i];
            M(0,0)= M(0,0) + (point[1]-x0[6])*(point[1]-x0[6]) + (point[2]-x0[7])*(point[2]-x0[7]);
            M(0,1)= M(0,1) - (point[1]-x0[6])*(point[0]-x0[5]);
            M(0,2)= M(0,2) - (point[2]-x0[7])*(point[0]-x0[5]);
            M(1,1)= M(1,1) + (point[0]-x0[5])*(point[0]-x0[5]) + (point[2]-x0[7])*(point[2]-x0[7]);
            M(2,2)= M(2,2) + (point[1]-x0[6])*(point[1]-x0[6]) + (point[0]-x0[5])*(point[0]-x0[5]);
            M(1,2)= M(1,2) - (point[2]-x0[7])*(point[1]-x0[6]);
        }

        M(0,0)= M(0,0)/point_cloud.size();
        M(0,1)= M(0,1)/point_cloud.size();
        M(0,2)= M(0,2)/point_cloud.size();
        M(1,1)= M(1,1)/point_cloud.size();
        M(2,2)= M(2,2)/point_cloud.size();
        M(1,2)= M(1,2)/point_cloud.size();

        M(1,0)= M(0,1);
        M(2,0)= M(0,2);
        M(2,1)= M(1,2);

        SVDJacobi(M,u,s,v);
        n=u.getCol(0);
        o=u.getCol(1);
        a=u.getCol(2);

        R.setCol(1,n);
        R.setCol(2,o);
        R.setCol(0,a);

        return R;
    }

    /****************************************************************/
    bool updateModule()
    {
        return false;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.0;
    }

    /****************************************************************/
    bool close()
    {

    }


};





/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Augmenter augment_pc;
    return augment_pc.runModule(rf);
}
