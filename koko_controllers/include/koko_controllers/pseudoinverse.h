#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::MatrixXd  pseudoinverse(const Eigen::MatrixXd &mat, double tolerance) // choose appropriately
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::MatrixXd &singularValues = svd.singularValues();
    Eigen::Matrix<double, Eigen::MatrixXd::ColsAtCompileTime, Eigen::MatrixXd::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = 1 / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = 0;
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}
