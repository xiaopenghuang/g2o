#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/factory.h>

#include <g2o/types/slam3d/vertex_se3.h>

#include <iostream>
#include <stdexcept>

using namespace std;
using namespace g2o;

G2O_USE_TYPE_GROUP(slam3d);

int main(int argc, char **argv)
{
	//create the linear solver
	BlockSolverX::LinearSolverType *linearSolver =  new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();

	// create the block solver on top of the linear solver
	BlockSolverX *blockSolver = new BlockSolverX(linearSolver);

	// create the algorithm to carry out the optimization
	OptimizationAlgorithmLevenberg *optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);

	// create the optimizer to load the data and carry out the optimization
	SparseOptimizer optimizer;

	try{
		optimizer.load("../data/sphere_bignoise_vertex3.g2o");
		cout << "load: " << optimizer.vertices().size() << " vertices " << endl;
		cout << "load: " << optimizer.edges().size() << " edges " << endl;
	}
	catch(const std::exception &e){
		throw std::runtime_error("can not load data");
	}

	VertexSE3 * firstVertex = dynamic_cast<VertexSE3*>(optimizer.vertex(0));
	firstVertex->setFixed(true);

	optimizer.setVerbose(true);
	optimizer.setAlgorithm(optimizationAlgorithm);

	optimizer.initializeOptimization();

	cout << "start optimization ..... " << endl;

	optimizer.optimize(10);

	cout << "finish optimization ..... "<< endl;

	optimizer.save("../data/after_optimizer.g2o");

	return 0;
}
