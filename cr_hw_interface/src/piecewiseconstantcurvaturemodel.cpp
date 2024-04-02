#include "cr_hw_interface/piecewiseconstantcurvaturemodel.h"
#include <chrono>

// Wrapper function for non-linear boundary conditions for GNU::GSL
int evaluate_boundary_conditions_pcc(const gsl_vector * x, void *param, gsl_vector * f)
{


    PiecewiseConstantCurvatureModel *model = (PiecewiseConstantCurvatureModel*)param;

    Eigen::MatrixXd inputs;

    inputs.resize(3*model->getNumberOfTotalDisks(),1);

    Eigen::MatrixXd outputs;

    for(int i = 0; i < 3*model->getNumberOfTotalDisks(); i++)
    {
        inputs(i,0) = gsl_vector_get (x, i);
    }



    model->get_res(outputs, inputs);




    for(int i = 0; i < 3*model->getNumberOfTotalDisks(); i++)
    {

        gsl_vector_set (f, i, outputs(i,0));
    }


    return GSL_SUCCESS;
}

// Callback function that can be used during non-linear squares solving
void
callback_pcc(const size_t iter, void *param,
         const gsl_multifit_nlinear_workspace *w)
{

    PiecewiseConstantCurvatureModel *model = (PiecewiseConstantCurvatureModel*)param;

    //Print current iteration
    std::cout << "Iteration " << iter << std::endl;

    gsl_vector * x = gsl_multifit_nlinear_position(w);
    gsl_vector * r = gsl_multifit_nlinear_residual(w);

    std::cout << "Current values:" << std::endl;
    for(int i = 0; i < 3*model->getNumberOfTotalDisks(); i++)
    {
        std::cout << " " << gsl_vector_get(x, i);
    }
    std::cout << std::endl << "Current residuals:" << std::endl;
    for(int i = 0; i < 3*model->getNumberOfTotalDisks(); i++)
    {
        std::cout << " " << gsl_vector_get(r, i);
    }

    double chisq;
    gsl_blas_ddot(r, r, &chisq);

    std::cout << std::endl << "Current cost: "<< chisq << std::endl << std::endl;

}

PiecewiseConstantCurvatureModel::PiecewiseConstantCurvatureModel()
{
	//Initialize member variables and set up default parameters for TDCR
    m_length[0]         = 0.13;
    m_length[1]         = 0.13;
    m_youngs_modulus    = 110e9;
    m_number_disks[0]   = 2;
    m_number_disks[1]   = 2;
    m_pradius_disks[0]  = 0.0035;
    m_pradius_disks[1]  = 0.0035;
    m_ro                = 0.0005;

    std::vector<Eigen::Vector3d> routing;

    Eigen::Vector3d tendon1;
    tendon1 << 0,
            m_pradius_disks[0],
            0;

    Eigen::Vector3d tendon2;
    tendon2 <<  m_pradius_disks[0],
            0,
            0;
    Eigen::Vector3d tendon3;
    tendon3 <<  0,
            -m_pradius_disks[0],
            0;
    Eigen::Vector3d tendon4;
    tendon4 <<  -m_pradius_disks[0],
            0,
            0;

    routing.push_back(tendon1);
    routing.push_back(tendon2);
    routing.push_back(tendon3);
    routing.push_back(tendon4);

    tendon1 << 0,
            m_pradius_disks[1],
            0;

    tendon2 <<  m_pradius_disks[1],
            0,
            0;

    tendon3 <<  0,
            -m_pradius_disks[1],
            0;

    tendon4 <<  -m_pradius_disks[1],
            0,
            0;

    routing.push_back(tendon1);
    routing.push_back(tendon2);
    routing.push_back(tendon3);
    routing.push_back(tendon4);

    m_routing           = routing;



    m_keep_inits = true;
    m_default_inits.resize(1,3*(m_number_disks[0]+m_number_disks[1]));
    for(int i = 0; i < m_default_inits.cols(); i++)
    {
        m_default_inits(0,i) = 0.01;
    }
    m_last_inits = m_default_inits;


    m_tau.setZero();

}

double PiecewiseConstantCurvatureModel::getNumberOfTotalDisks()
{
    return m_number_disks[0] + m_number_disks[1];
}

void PiecewiseConstantCurvatureModel::setDefaultInitValues(Eigen::MatrixXd inits)
{
    m_default_inits = inits;
}

Eigen::MatrixXd PiecewiseConstantCurvatureModel::getFinalInitValues()
{
    return m_last_inits;
}

double PiecewiseConstantCurvatureModel::getFinalResudial()
{
    return m_residual_error;
}

Eigen::Matrix4d PiecewiseConstantCurvatureModel::getDiskTransform(Eigen::MatrixXd var, Eigen::VectorXd length_ss, int idx_from, int idx_to)
{
    Eigen::Matrix4d diskTransform = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d tempTransform = Eigen::Matrix4d::Identity();


    for(int i = idx_to + 1; i <= idx_from; i++)
    {
        double beta = var(3*i-3);
        double gamma = var(3*i-2);
        double epsi = var(3*i-1);
        double k = std::sqrt(beta*beta+gamma*gamma);
        double phi = std::atan2(gamma,beta);
        double theta = k*length_ss(i-1);
        Eigen::Vector3d p;
        p <<    (1-std::cos(theta))*std::cos(phi)/k,
                (1-std::cos(theta))*std::sin(phi)/k,
                std::sin(theta)/k;
        Eigen::Matrix3d Rz;
        Rz << std::cos(phi), -1*std::sin(phi), 0,
              std::sin(phi), std::cos(phi), 0,
              0, 0, 1;
        Eigen::Matrix3d Ry;
        Ry << std::cos(theta), 0, std::sin(theta),
              0, 1, 0,
              -1*std::sin(theta), 0, std::cos(theta);
        Eigen::Matrix3d Rz2;
        Rz2 << std::cos(epsi-phi), -1*std::sin(epsi-phi), 0,
              std::sin(epsi-phi), std::cos(epsi-phi), 0,
              0, 0, 1;
        tempTransform << Rz*Ry*Rz2, p,
                         0, 0, 0, 1;

        diskTransform = diskTransform*tempTransform;

    }

    return diskTransform;

}

PiecewiseConstantCurvatureModel::~PiecewiseConstantCurvatureModel()
{

}


bool PiecewiseConstantCurvatureModel::forwardKinematics(Eigen::MatrixXd &diskFrames, Eigen::MatrixXd q, Eigen::Vector3d f_ext, Eigen::Vector3d l_ext)
{
    m_tau = q;
    m_f_ext = f_ext;
    m_l_ext = l_ext;

    // Run BVP Shooting method
    Eigen::MatrixXd inits;

    if(m_keep_inits)
    {
        inits = m_last_inits;
    }
    else
    {
        inits = m_default_inits;
    }

    // gsl nonlinear equation solver referrence: https://www.gnu.org/software/gsl/doc/html/nls.html#examples
    const size_t n = 3*(m_number_disks[0]+m_number_disks[1]); //Number of equations (boundary conditions)
    const size_t p = 3*(m_number_disks[0]+m_number_disks[1]); //Number of parameters (initial guesses)
    gsl_vector *x0 = gsl_vector_alloc(p);
    gsl_multifit_nlinear_fdf fdf;
    gsl_multifit_nlinear_parameters fdf_params = gsl_multifit_nlinear_default_parameters();

    fdf.f = evaluate_boundary_conditions_pcc;
    fdf.df = NULL; //Provide your own Jacobian
    fdf.fvv = NULL; //Provide your own Hessian
    fdf.n = n;
    fdf.p = p;
    fdf.params = this;

    //Set initial values
    for(int i = 0; i < 3*(m_number_disks[0]+m_number_disks[1]); i++)
    {
        gsl_vector_set(x0,i,inits(0,i));
    }

    //Choose solver (Levenberg-Marquardt)
    //fdf_params.trs = gsl_multifit_nlinear_trs_lm;
    //Choose solver (Levenberg-Marquardt with acceleration)
    //fdf_params.trs = gsl_multifit_nlinear_trs_lmaccel;
    //Choose solver (dogleg)
    fdf_params.trs =  gsl_multifit_nlinear_trs_dogleg;

    //Set scaling parameter
    fdf_params.scale =   gsl_multifit_nlinear_scale_more;

    //Set solver for trust region
    fdf_params.solver =  gsl_multifit_nlinear_solver_qr;

    //Set trust region factor
    fdf_params.factor_up = 2.5; //default 3
    fdf_params.factor_down = 4; //default 2


    ///---Solve the system---
    const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
    const size_t max_iter = 1000;
    const double xtol = 1.0e-6;
    const double gtol = 1.0e-6;
    const double ftol = 1.0e-6;
    gsl_multifit_nlinear_workspace *work = gsl_multifit_nlinear_alloc(T,&fdf_params, n, p);
    gsl_vector * f = gsl_multifit_nlinear_residual(work);
    gsl_vector * x = gsl_multifit_nlinear_position(work);
    int info;
    double chisq0, chisq, rcond;
    /* initialize solver */
    gsl_multifit_nlinear_init(x0, &fdf, work);
    /* store initial cost */
    gsl_blas_ddot(f, f, &chisq0);

    /* iterate until convergence */
    //gsl_multifit_nlinear_driver(max_iter, xtol, gtol, ftol,callback_pcc, this, &info, work);
    //Without Callback
    gsl_multifit_nlinear_driver(max_iter, xtol, gtol, ftol,NULL, NULL, &info, work);

    /* store final cost */
    gsl_blas_ddot(f, f, &chisq);
    m_residual_error = chisq;

    /* store cond(J(x)) */
    gsl_multifit_nlinear_rcond(&rcond, work);

    ///---End Solve the system---
    Eigen::MatrixXd final_values;
    final_values.resize(1,3*(m_number_disks[0]+m_number_disks[1]));

    Eigen::MatrixXd final_boundcon;
    final_boundcon.resize(1,3*(m_number_disks[0]+m_number_disks[1]));


    for(int i = 0; i < 3*(m_number_disks[0]+m_number_disks[1]); i ++)
    {
        final_values(0,i) = gsl_vector_get (x, i);
        final_boundcon(0,i) = gsl_vector_get (f, i);
    }

    m_last_inits = final_values;
    gsl_vector_free (x0);
    gsl_multifit_nlinear_free(work);

    Eigen::VectorXd length_ss;
    length_ss.resize(m_number_disks[0]+m_number_disks[1]);

    for(int i = 0; i < m_number_disks[0]; i++)
    {
        length_ss(i) = m_length[0]/m_number_disks[0];
    }

    for(int i = m_number_disks[0]; i < m_number_disks[0]+m_number_disks[1]; i++)
    {
        length_ss(i) = m_length[1]/m_number_disks[1];
    }

    diskFrames.resize(4,4*(m_number_disks[0]+m_number_disks[1]+1));

    diskFrames.block(0,0,4,4) = Eigen::Matrix4d::Identity();

    for(int i = 1; i < m_number_disks[0]+m_number_disks[1] + 1; i++)
    {
        Eigen::Matrix4d frame = getDiskTransform(final_values.transpose(),length_ss,i,0);
        diskFrames.block(0,4*i,4,4) = frame;
    }

    if(m_residual_error > 1e-6)
    {
        return false;
    }

    return true;

}

// net force and moment equilibirium for solving the static model
void PiecewiseConstantCurvatureModel::get_res(Eigen::MatrixXd &output, Eigen::MatrixXd input)
{

    Eigen::VectorXd length_ss;
    length_ss.resize(m_number_disks[0]+m_number_disks[1]);

    for(int i = 0; i < m_number_disks[0]; i++)
    {
        length_ss(i) = m_length[0]/m_number_disks[0];
    }

    for(int i = m_number_disks[0]; i < m_number_disks[0]+m_number_disks[1]; i++)
    {
        length_ss(i) = m_length[1]/m_number_disks[1];
    }

    output.resize(3*(m_number_disks[0]+m_number_disks[1]),1);

    Eigen::Vector4d F_prev;
    F_prev.setZero();
    Eigen::Vector3d M_prev;
    M_prev.setZero();

    for(int ss_i = m_number_disks[0]+m_number_disks[1]; ss_i > 0; ss_i--)
    {

        //Get transformation matrix between disk i and disk i-1
        Eigen::Matrix4d diskTransform;
        double beta = input(3*ss_i-3);
        double gamma = input(3*ss_i-2);
        double epsi = input(3*ss_i-1);
        double k = std::sqrt(beta*beta+gamma*gamma);
        double phi = std::atan2(gamma,beta);
        double theta = k*length_ss(ss_i-1);
        Eigen::Vector3d p;
        p <<    (1-std::cos(theta))*std::cos(phi)/k,
                (1-std::cos(theta))*std::sin(phi)/k,
                std::sin(theta)/k;
        Eigen::Matrix3d Rz;
        Rz << std::cos(phi), -1*std::sin(phi), 0,
              std::sin(phi), std::cos(phi), 0,
              0, 0, 1;
        Eigen::Matrix3d Ry;
        Ry << std::cos(theta), 0, std::sin(theta),
              0, 1, 0,
              -1*std::sin(theta), 0, std::cos(theta);
        Eigen::Matrix3d Rz2;
        Rz2 << std::cos(epsi-phi), -1*std::sin(epsi-phi), 0,
              std::sin(epsi-phi), std::cos(epsi-phi), 0,
              0, 0, 1;
        diskTransform << Rz*Ry*Rz2, p,
                         0, 0, 0, 1;

        Eigen::Matrix<double,3,8> tendon_pos;
        for(int i = 0; i < 8; i++)
        {
            tendon_pos.col(i) = diskTransform.block(0,0,3,3)*m_routing.at(i) + diskTransform.block(0,3,3,1);
        }

        Eigen::MatrixXd pt_mat;
        pt_mat.resize(4,8);
        pt_mat << m_routing.at(0) - tendon_pos.col(0), m_routing.at(1) - tendon_pos.col(1), m_routing.at(2) - tendon_pos.col(2), m_routing.at(3) - tendon_pos.col(3), m_routing.at(4) - tendon_pos.col(4), m_routing.at(5) - tendon_pos.col(5), m_routing.at(6) - tendon_pos.col(6), m_routing.at(7) - tendon_pos.col(7),
                  0, 0, 0, 0, 0, 0, 0, 0;

        Eigen::VectorXd norm_ct1;
        norm_ct1.resize(8);
        for(int i = 0; i < 8; i++)
        {
            norm_ct1(i) = (m_routing.at(i) - tendon_pos.col(i)).norm();
        }

        Eigen::MatrixXd ct1_mat;
        ct1_mat.resize(4,8);
        ct1_mat << norm_ct1.transpose(),
                   norm_ct1.transpose(),
                   norm_ct1.transpose(),
                   norm_ct1.transpose();

        Eigen::MatrixXd F_disk;
        F_disk.resize(m_number_disks[0]+m_number_disks[1],8);
        F_disk.setZero();

        for(int i = 0; i < m_number_disks[0]; i++)
        {
            F_disk.block(i,0,1,8) = m_tau.transpose();
        }

        for(int i = m_number_disks[0]; i < m_number_disks[0] + m_number_disks[1]; i++)
        {
            F_disk.block(i,4,1,4) = m_tau.block(4,0,4,1).transpose();
        }

        Eigen::MatrixXd F_disk_mat;
        F_disk_mat.resize(4,8);
        F_disk_mat << F_disk.row(ss_i-1),
                      F_disk.row(ss_i-1),
                      F_disk.row(ss_i-1),
                      F_disk.row(ss_i-1);

        Eigen::MatrixXd F_rel;
        F_rel.resize(4,8);
        F_rel.setZero();

        Eigen::MatrixXd M_rel;
        M_rel.resize(3,8);
        M_rel.setZero();

        Eigen::Vector4d zi = diskTransform.col(2);

        if(ss_i < m_number_disks[0]+m_number_disks[1])
        {
            Eigen::Matrix4d tempTransform = getDiskTransform(input,length_ss,ss_i+1,ss_i-1);

            Eigen::Matrix<double,3,8> tendon_pos_temp;
            for(int i = 0; i < 8; i++)
            {
                tendon_pos_temp.col(i) = tempTransform.block(0,0,3,3)*m_routing.at(i) + tempTransform.block(0,3,3,1);
            }

            Eigen::MatrixXd pt1_mat;
            pt1_mat.resize(4,8);
            pt1_mat <<tendon_pos_temp.col(0) - tendon_pos.col(0), tendon_pos_temp.col(1) - tendon_pos.col(1), tendon_pos_temp.col(2) - tendon_pos.col(2), tendon_pos_temp.col(3) - tendon_pos.col(3), tendon_pos_temp.col(4) - tendon_pos.col(4),tendon_pos_temp.col(5) - tendon_pos.col(5),tendon_pos_temp.col(6) - tendon_pos.col(6),tendon_pos_temp.col(7) - tendon_pos.col(7),
                      0, 0, 0, 0, 0, 0, 0, 0;


            Eigen::VectorXd norm_ct2;
            norm_ct2.resize(8);
            for(int i = 0; i < 8; i++)
            {
                norm_ct2(i) = (tendon_pos_temp.col(i)- tendon_pos.col(i)).norm();
            }

            Eigen::MatrixXd ct2_mat;
            ct2_mat.resize(4,8);
            ct2_mat << norm_ct2.transpose(),
                       norm_ct2.transpose(),
                       norm_ct2.transpose(),
                       norm_ct2.transpose();

            Eigen::MatrixXd F_disk_mat1;
            F_disk_mat1.resize(4,8);
            F_disk_mat1 << F_disk.row(ss_i),
                          F_disk.row(ss_i),
                          F_disk.row(ss_i),
                          F_disk.row(ss_i);

            F_rel = pt_mat.array()/ct1_mat.array()*F_disk_mat.array() + pt1_mat.array()/ct2_mat.array()*F_disk_mat1.array();
            if(ss_i == m_number_disks[0])
            {
                for(int i = 4; i < 8; i++)
                {
                    F_rel.col(i) = F_rel.col(i) - zi.dot(F_rel.col(i))*zi/zi.norm()/zi.norm();
                }

            }
            else
            {
                for(int i = 0; i < 8; i++)
                {
                    F_rel.col(i) = F_rel.col(i) - zi.dot(F_rel.col(i))*zi/zi.norm()/zi.norm();
                }
            }

        }
        else if(ss_i == m_number_disks[0]+m_number_disks[1])
        {
            F_rel = pt_mat.array()/ct1_mat.array()*F_disk_mat.array();
            F_prev.setZero();
            M_prev.setZero();

        }

        for(int i = 0; i < 8; i++)
        {
            Eigen::Vector3d pos = tendon_pos.col(i);
            Eigen::Vector3d F = F_rel.block(0,i,3,1);
            M_rel.col(i) = pos.cross(F);
        }

        Eigen::Vector4d F_ext;
        F_ext.setZero();
        Eigen::Vector3d M_ext;
        M_ext.setZero();
        if(ss_i == m_number_disks[0]+m_number_disks[1])
        {
            Eigen::Matrix4d Rt = getDiskTransform(input,length_ss,ss_i-1,0);

            F_ext = Rt.inverse()*Eigen::Vector4d(m_f_ext(0),m_f_ext(1),m_f_ext(2),0);
            Eigen::Vector3d F_ext_temp = F_ext.block(0,0,3,1);
            M_ext = Rt.block(0,0,3,3).transpose()*m_l_ext + p.cross(F_ext_temp);
        }

        F_prev = diskTransform*F_prev;
        M_prev = diskTransform.block(0,0,3,3)*M_prev;

        Eigen::Vector3d F_ext_temp = F_prev.block(0,0,3,1);

        Eigen::Vector4d F_net = F_rel.rowwise().sum() + F_prev + F_ext;
        F_net(3) = 0;
        Eigen::Vector3d M_net = M_rel.rowwise().sum() + M_prev + M_ext + p.cross(F_ext_temp);


        double I = 0.25*M_PI*(m_ro*m_ro*m_ro*m_ro);
        double G = m_youngs_modulus/(2*(1.3));

        Eigen::Vector3d M_bend = Rz*Eigen::Vector3d(0,k*m_youngs_modulus*I,0);
        Eigen::Vector3d M_tor = diskTransform.block(0,0,3,3)*Eigen::Vector3d(0, 0, 2*I*G*epsi/length_ss(ss_i-1));

        output.block(3*ss_i-3,0,3,1) = M_bend + M_tor - M_net;

        F_prev = F_net;
        M_prev = M_net;

    }
}

void PiecewiseConstantCurvatureModel::setKeepInits(bool keep)
{
    m_keep_inits = keep;
}


Eigen::MatrixXd PiecewiseConstantCurvatureModel::getTendonDisplacements(Eigen::MatrixXd m_disk_frames)
{
    Eigen::MatrixXd tendon_length;
    tendon_length.resize(8,1);
    tendon_length.setZero();
    //Iterate through all the disk frames
    for(int k = 1; k < this -> getNumberOfTotalDisks()+1; k++)
    {
        Eigen::Matrix4d disk_frame;
        disk_frame = m_disk_frames.block(0,4*k,4,4);
        Eigen::Matrix4d disk_frame_prev;
        disk_frame_prev = m_disk_frames.block(0,4*(k-1),4,4);
        Eigen::Matrix4d routing1 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d routing2 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d routing3 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d routing4 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d routing5 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d routing6 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d routing7 = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d routing8 = Eigen::Matrix4d::Identity();

        routing1.block(0,3,3,1) = m_routing.at(0);
        routing2.block(0,3,3,1) = m_routing.at(1);
        routing3.block(0,3,3,1) = m_routing.at(2);
        routing4.block(0,3,3,1) = m_routing.at(3);
        routing5.block(0,3,3,1) = m_routing.at(4);
        routing6.block(0,3,3,1) = m_routing.at(5);
        routing7.block(0,3,3,1) = m_routing.at(6);
        routing8.block(0,3,3,1) = m_routing.at(7);

        if(k < m_number_disks[0] + 1)
        {
            tendon_length(0) += ((disk_frame*routing1).block(0,3,3,1) - (disk_frame_prev*routing1).block(0,3,3,1)).norm();
            tendon_length(1) += ((disk_frame*routing2).block(0,3,3,1) - (disk_frame_prev*routing2).block(0,3,3,1)).norm();
            tendon_length(2) += ((disk_frame*routing3).block(0,3,3,1) - (disk_frame_prev*routing3).block(0,3,3,1)).norm();
            tendon_length(3) += ((disk_frame*routing4).block(0,3,3,1) - (disk_frame_prev*routing4).block(0,3,3,1)).norm();
        }
        else
        {
            tendon_length(4) += ((disk_frame*routing5).block(0,3,3,1) - (disk_frame_prev*routing5).block(0,3,3,1)).norm();
            tendon_length(5) += ((disk_frame*routing6).block(0,3,3,1) - (disk_frame_prev*routing6).block(0,3,3,1)).norm();
            tendon_length(6) += ((disk_frame*routing7).block(0,3,3,1) - (disk_frame_prev*routing7).block(0,3,3,1)).norm();
            tendon_length(7) += ((disk_frame*routing8).block(0,3,3,1) - (disk_frame_prev*routing8).block(0,3,3,1)).norm();
        }

    }

    tendon_length(0) = m_length[0] - tendon_length(0);
    tendon_length(1) = m_length[0] - tendon_length(1);
    tendon_length(2) = m_length[0] - tendon_length(2);
    tendon_length(3) = m_length[0] - tendon_length(3);

    tendon_length(4) = m_length[1] - tendon_length(4);
    tendon_length(5) = m_length[1] - tendon_length(5);
    tendon_length(6) = m_length[1] - tendon_length(6);
    tendon_length(7) = m_length[1] - tendon_length(7);

    return tendon_length;
}

// set robot parameter directly here
void PiecewiseConstantCurvatureModel::setdefaultParameter(double spacer_distance, double dist_to_tendon, double spacer_num, double segment_num)
{
    m_length[0]         = 0.13;
    m_length[1]         = 0.13;
    m_youngs_modulus    = 50*1e9;
    // compare to real robot model is the model disk number of 1-4, 1-8, 2-4, 2-8
    m_number_disks[0]   = 2;
    m_number_disks[1]   = 2; 
    m_pradius_disks[0]  = dist_to_tendon;
    m_pradius_disks[1]  = dist_to_tendon;
    m_ro                = 0.0005; // radiu of robot's backbone
    Eigen::Vector3d tendon1;
    tendon1 << 0,
            m_pradius_disks[0],
            0;
    Eigen::Vector3d tendon2;
    tendon2 <<  m_pradius_disks[0],
            0,
            0;
    Eigen::Vector3d tendon3;
    tendon3 <<  0,
            -m_pradius_disks[0],
            0;
    Eigen::Vector3d tendon4;
    tendon4 <<  -m_pradius_disks[0],
            0,
            0;
    m_routing.push_back(tendon1);
    m_routing.push_back(tendon2);
    m_routing.push_back(tendon3);
    m_routing.push_back(tendon4);

    tendon1 << 0,
            m_pradius_disks[1],
            0;
    tendon2 <<  m_pradius_disks[1],
            0,
            0;
    tendon3 <<  0,
            -m_pradius_disks[1],
            0;
    tendon4 <<  -m_pradius_disks[1],
            0,
            0;
    m_routing.push_back(tendon1);
    m_routing.push_back(tendon2);
    m_routing.push_back(tendon3);
    m_routing.push_back(tendon4);
    
}


position_cfg PiecewiseConstantCurvatureModel::calculate_pose(Eigen::Matrix4d frame)
{
    Eigen::Matrix3d rotation = frame.topLeftCorner<3,3>();
    double alpha,beta,gamma;
    double r00 = rotation(0,0);
    double r01 = rotation(0,1);
    double r02 = rotation(0,2);
    double r10 = rotation(1,0);
    double r11 = rotation(1,1);
    double r12 = rotation(1,2);
    double r20 = rotation(2,0);
    double r21 = rotation(2,1);
    double r22 = rotation(2,2);
    position_cfg pose;
    // Split rotation matrix into angles: R_zR_yR_z. Source: https://www.geometrictools.com/Documentation/EulerAngles.pdf
    if(r22<1)
    {
        if(r22>-1)
        {
            alpha = acos(r22);
            beta = atan2(r12, r02);
            gamma = atan2(r21, -r20);
        }
        else // r22 = -1
        {
            //Not a unique solution: thetaZ1−thetaZ0 = atan2(r10,r11)
            alpha = M_PI;
            beta = -atan2(r10, r11);
            gamma = 0;
        }
    }
    else // r22 = 1
    {
        //Not a unique solution: thetaZ1+thetaZ0 = atan2(r10,r11)
        alpha = 0;
        beta = atan2(r10, r11);
        gamma = 0;
    }
    pose.x = frame.rightCols(1)(0);
    pose.y = frame.rightCols(1)(1);
    pose.z = frame.rightCols(1)(2);
    pose.alpha = alpha;
    pose.beta = beta;
    pose.gamma = gamma;


    return pose;
}

position_cfg PiecewiseConstantCurvatureModel::get_segment_pose(Eigen::MatrixXd diskframes,int i)
{
    Eigen::Matrix4d frame = diskframes.block(0,4*i,4,4);
    Eigen::Matrix3d rotation = frame.topLeftCorner<3,3>();
    double r00 = rotation(0,0);
    double r01 = rotation(0,1);
    double r02 = rotation(0,2);
    double r10 = rotation(1,0);
    double r11 = rotation(1,1);
    double r12 = rotation(1,2);
    double r20 = rotation(2,0);
    double r21 = rotation(2,1);
    double r22 = rotation(2,2);
    double alpha,beta,gamma;
    position_cfg pose;
    pose.x = frame.rightCols(1)(0);
    pose.y = frame.rightCols(1)(1);
    pose.z = frame.rightCols(1)(2);
    if(r22<1)
    {
        if(r22>-1)
        {
            alpha = acos(r22);
            beta = atan2(r12, r02);
            gamma = atan2(r21, -r20);
        }
        else // r22 = -1
        {
            //Not a unique solution: thetaZ1−thetaZ0 = atan2(r10,r11)
            alpha = M_PI;
            beta = -atan2(r10, r11);
            gamma = 0;
        }
    }
    else // r22 = 1
    {
        //Not a unique solution: thetaZ1+thetaZ0 = atan2(r10,r11)
        alpha = 0;
        beta = atan2(r10, r11);
        gamma = 0;
    }
    pose.alpha = alpha;
    pose.beta = beta;
    pose.gamma = gamma;
    return pose;
}

double PiecewiseConstantCurvatureModel::get_segmentx(Eigen::Matrix4d pre, Eigen::Matrix4d current, int sectionNum)
{
    Eigen::Matrix3d pre_rotation = pre.topLeftCorner<3,3>();
    Eigen::Matrix3d current_rotation = current.topLeftCorner<3,3>();
    double pre_x,current_x;

    double r02 = pre_rotation(0,2);
    double r10 = pre_rotation(1,0);
    double r11 = pre_rotation(1,1);
    double r12 = pre_rotation(1,2);
    double r22 = pre_rotation(2,2);
    // Split rotation matrix into angles: R_xR_yR_z. Source: https://www.geometrictools.com/Documentation/EulerAngles.pdf
    if(r02<1)
    {
        if(r02>-1)
        {
            pre_x = atan2(-r12, r22);
        }
        else // r02 = -1
        {
            //Not a unique solution: thetaZ−thetaX = atan2(r10,r11)
            pre_x = -atan2(r10, r11);
        }
    }
    else // r02 = 1
    {
        //Not a unique solution: thetaZ+thetaX = atan2(r10,r11)
        pre_x = atan2(r10, r11);
    }

    r02 = current_rotation(0,2);
    r10 = current_rotation(1,0);
    r11 = current_rotation(1,1);
    r12 = current_rotation(1,2);
    r22 = current_rotation(2,2);
    if(r02<1)
    {
        if(r02>-1)
        {
            current_x = atan2(-r12, r22);
        }
        else // r02 = -1
        {
            current_x = -atan2(r10, r11);
        }
    }
    else // r02 = 1
    {
        //Not a unique solution: thetaZ+thetaX = atan2(r10,r11)
        current_x = atan2(r10, r11);
    }

    return (current_x - pre_x)/sectionNum;
}
double PiecewiseConstantCurvatureModel::get_segmenty(Eigen::Matrix4d pre, Eigen::Matrix4d current, int sectionNum)
{
    Eigen::Matrix3d pre_rotation = pre.topLeftCorner<3,3>();
    Eigen::Matrix3d current_rotation = current.topLeftCorner<3,3>();
    double pre_y,current_y;


    double r02 = pre_rotation(0,2);
    // Split rotation matrix into angles: R_xR_yR_z. Source: https://www.geometrictools.com/Documentation/EulerAngles.pdf
    if(r02<1)
    {
        if(r02>-1)
        {
            pre_y = asin(r02);
        }
        else // r02 = -1
        {
            //Not a unique solution: thetaZ−thetaX = atan2(r10,r11)
            pre_y = -M_PI/2;
        }
    }
    else // r02 = 1
    {
        //Not a unique solution: thetaZ+thetaX = atan2(r10,r11)
        pre_y = M_PI/2;
    }

    r02 = current_rotation(0,2);
    if(r02<1)
    {
        if(r02>-1)
        {
            current_y = asin(r02);
        }
        else // r02 = -1
        {
            //Not a unique solution: thetaZ−thetaX = atan2(r10,r11)
            current_y = -M_PI/2;
        }
    }
    else // r02 = 1
    {
        //Not a unique solution: thetaZ+thetaX = atan2(r10,r11)
        current_y = M_PI/2;
    }
    
    return (current_y - pre_y)/sectionNum;
}
double PiecewiseConstantCurvatureModel::get_segmentz(position_cfg pre, position_cfg current, int sectionNum)
{
    return (current.z - pre.z)/sectionNum - (this -> m_length[0]/8);
}