#include <pybind11/pybind11.h>

#include <solver_ipft/util/debug.hpp>

#include "py_conversions.hpp"
#include "py_ipft_solver.hpp"
#include "py_node.hpp"
#include "py_particle_belief.hpp"
#include "py_planner.hpp"
#include "py_planner_cld.hpp"

namespace py = pybind11;

namespace solver_ipft {
namespace pyadapters {

/* --------------------------- py_conversions.cpp --------------------------- */

py::array_t<double> point2NpArray(const Point& p) {
    int dim = p.dimensions();
    py::array_t<double> npArr;
    npArr.resize({dim});
    auto npArrData = npArr.mutable_unchecked<1>();
    for (int i = 0; i < dim; i++) {
        npArrData(i) = p.get(i);
    }
    return std::move(npArr);
}

py::array_t<double> action2NpArray(const Action& act, const POMDP* model) {
    auto actionValue = model->valueOfAction(act);
    return point2NpArray(*(actionValue.get()));
}

py::array_t<double> value2NpArray(const Value& value) {
    int dim = value.getComponentCount();
    py::array_t<double> npArr;
    npArr.resize({dim});
    auto npArrData = npArr.mutable_unchecked<1>();
    for (int i = 0; i < dim; i++) {
        npArrData(i) = value.getWeightedComponent(i);
    }
    return std::move(npArr);
}

void particleSet2NpArrays(const std::vector<State*>& particleSet, py::array_t<double>& particles, py::array_t<double>& weights) {
    CHECK_GT(particleSet.size(), 0) << "Particle set empty!";
    int num_particles = particleSet.size();
    int dim = particleSet[0]->dimensions();

    // resize np arrays
    particles.resize({num_particles, dim});
    weights.resize({num_particles});

    // data accessors
    auto particlesData = particles.mutable_unchecked<2>();
    auto weightsData = weights.mutable_unchecked<1>();

    for (int i = 0; i < num_particles; i++) {
        // particle
        State* particle = particleSet[i];
        for (int j = 0; j < dim; j++) {
            particlesData(i, j) = particle->get(j);
        }
        // weight
        weightsData(i) = particle->weight_;
    }
}

// particle np array: one row corresponds to a state, columns correspond to the dimensions
std::vector<State*> npArray2ParticleSet(const py::array_t<double>& unweightedParticleArray, const POMDP* model) {
    // check array dimensions
    if (unweightedParticleArray.ndim() != 2) {
        throw std::invalid_argument(std::string("Number of dimension must be 2, but is ") +
                                    std::to_string(unweightedParticleArray.ndim()));
    }
    // check if shape of array matches model
    int arrStateDim = unweightedParticleArray.shape(1);
    int modelStateDim = model->numDimStateSpace();
    if (arrStateDim != modelStateDim) {
        throw std::invalid_argument(std::string("Numpy array state dimension ") +
                                    std::to_string(arrStateDim) +
                                    std::string("does not match model state space dimension " +
                                                std::to_string(modelStateDim)));
    }
    // convert np array
    auto unwParticleArrData = unweightedParticleArray.unchecked<2>();
    int numParticles = unwParticleArrData.shape(0);  // number of rows of the array
    int numDims = unwParticleArrData.shape(1);       // number of columns of the array

    std::vector<State*> unwParticles;
    for (int r = 0; r < numParticles; r++) {
        State* s = model->allocateState();
        for (int c = 0; c < numDims; c++) {
            s->set(unwParticleArrData(r, c), c);
        }
        unwParticles.push_back(s);
    }
    // normalize particle weights
    State::normalizeWeights(unwParticles);
    return std::move(unwParticles);
}

/* ------------------------------- py_node.cpp ------------------------------ */

/* -------------------------------------------------------------------------- */
/*                                   PyNode                                   */
/* -------------------------------------------------------------------------- */

void PyNode::convertNode(std::shared_ptr<PyNode>& pynode, const Node* node) {
    pynode->count_ = node->getCount();
    pynode->treelevel_ = node->getTreelevel();
    const Value* val = node->getValueRef();
    if (val != nullptr)
        pynode->values_ = value2NpArray(*val);
}

/* -------------------------------------------------------------------------- */
/*                                   PyVNode                                  */
/* -------------------------------------------------------------------------- */

std::shared_ptr<PyVNode> PyVNode::convertTree(const VNode* vroot) {
    CHECK(vroot->getParent() == nullptr) << "Given VNode is not a root!";
    return convertVNode(vroot, nullptr);
}

std::shared_ptr<PyVNode> PyVNode::convertVNode(const VNode* vnode, const std::shared_ptr<PyQNode>& qparent) {
    auto pyvnode(std::make_shared<PyVNode>());
    auto pynode = std::static_pointer_cast<PyNode>(pyvnode);
    convertNode(pynode, vnode);
    pyvnode->parent_ = qparent;
    if (vnode->obsEdge_ != nullptr)
        pyvnode->observation_ = point2NpArray(*(vnode->obsEdge_));
    // TODO support general belief
    ParticleBelief* pbelief = static_cast<ParticleBelief*>(vnode->belief_);
    pyvnode->belief_ = PyParticleBelief::createPyParticleBelief(*pbelief);
    std::vector<QNode*> vNodeChildren = vnode->children();
    for (QNode* qchild : vNodeChildren) {
        pyvnode->children_.append(PyQNode::convertQNode(qchild, pyvnode));
    }
    return pyvnode;
}

/* -------------------------------------------------------------------------- */
/*                                   PyQNode                                  */
/* -------------------------------------------------------------------------- */

std::shared_ptr<PyQNode> PyQNode::convertQNode(const QNode* qnode, const std::shared_ptr<PyVNode>& vparent) {
    auto pyqnode(std::make_shared<PyQNode>());
    auto pynode = std::static_pointer_cast<PyNode>(pyqnode);
    convertNode(pynode, qnode);
    pyqnode->parent_ = vparent;
    pyqnode->action_ = action2NpArray(qnode->getAction(), qnode->model_);
    std::vector<VNode*> qNodeChildren = qnode->children();
    for (VNode* vchild : qNodeChildren) {
        pyqnode->children_.append(PyVNode::convertVNode(vchild, pyqnode));
    }
    return pyqnode;
}

/* ------------------------- py_particle_belief.cpp ------------------------- */

PyParticleBelief PyParticleBelief::createPyParticleBelief(const ParticleBelief& pb) {
    PyParticleBelief pyPb;
    if (!pb.particles_.empty())
        particleSet2NpArrays(pb.particles_, pyPb.particles_, pyPb.weights_);
    if (!pb.weighted_posterior_particles_.empty())
        particleSet2NpArrays(pb.weighted_posterior_particles_, pyPb.post_particles_, pyPb.post_weights_);
    pyPb.num_particles_ = pb.numParticles();
    return std::move(pyPb);
}

/* ----------------------------- py_planner.cpp ----------------------------- */

void PyPlanner::resetPlanner() {
    delete this->sim_stats_;
    search_trees_.clear();
    Planner::resetPlanner();
}

bool PyPlanner::runStep() {
    ValuedAction valuedAct = findAction();
    // get search tree of solver
    const VNode* root = this->solver_->getSearchTree();
    // convert C++ tree in Python tree
    auto pyroot = PyVNode::convertTree(root);
    // add tree to search_trees
    search_trees_.push_back(pyroot);
    bool terminal = takeAction(valuedAct);
    return terminal;
}

std::shared_ptr<PyVNode> PyPlanner::getSearchTreeOfStep(int i) const {
    if (i >= this->numSteps()) {
        throw std::invalid_argument(std::string("Maximum step index is ") + std::to_string(numSteps() - 1));
    }
    return this->search_trees_.at(i);  // access with bounds checking
}

int PyPlanner::numSteps() const {
    return this->step_;
}

/* --------------------------- py_ipft_solver.cpp --------------------------- */

/* -------------------------------------------------------------------------- */
/*                                PyIpft class                                */
/* -------------------------------------------------------------------------- */

void PyIpft::initializeSolver() {
    this->ipft_ = new Ipft(this->model_, this->initialBelief_, this->rand_, new BeliefInformationPolicy(this->model_, this->rand_));
}

void PyIpft::reset() {
    ParticleBelief* bel = static_cast<ParticleBelief*>(this->initialBelief_->clone());
    this->ipft_->setBelief(bel);
}

std::shared_ptr<PyVNode> PyIpft::getSearchTree() const {
    const VNode* root = this->ipft_->getSearchTree();
    if (root != nullptr) {
        return PyVNode::convertTree(root);
    } else {
        return nullptr;
    }
}

py::tuple PyIpft::search() {
    ValuedAction valuedAct = this->ipft_->search();
    // return values
    py::array_t<double> act = action2NpArray(valuedAct.action_, this->model_);
    py::array_t<double> value = value2NpArray(*(valuedAct.value_));
    double totalValue = valuedAct.value_->total();
    py::tuple ret = py::make_tuple(std::move(act), totalValue, std::move(value));

    // print results
    SearchStatistics* searchStats = this->ipft_->getSearchStatistics();
    LOG(INFO) << searchStats->text() << std::endl;
    delete searchStats;
    return std::move(ret);
}

PyParticleBelief PyIpft::getBelief() const {
    Belief* bel = this->ipft_->getBelief();
    PyParticleBelief pybel = PyParticleBelief::createPyParticleBelief(*static_cast<ParticleBelief*>(bel));
    delete bel;
    return std::move(pybel);
}

/* -------------------------------------------------------------------------- */
/*                               PyIpftCld class                              */
/* -------------------------------------------------------------------------- */

POMDP* PyIpftCld::initializeModel() const {
    POMDP* model = new cld::ContLightDark(this->rand_);
    return model;
}

/* -------------------------------------------------------------------------- */
/*                                test classes                                */
/* -------------------------------------------------------------------------- */

class PyConversionTest {
   private:
    Random* rand_;
    POMDP* model_;

    ParticleBelief* belief1;

   public:
    PyConversionTest() {
        this->rand_ = new Random();
        this->model_ = new cld::ContLightDark(rand_);

        std::vector<double> numbers = {4.253895,
                                       3.168835,
                                       3.300457,
                                       1.848377,
                                       -3.348563,
                                       -0.461945,
                                       2.089914,
                                       5.221298,
                                       0.026519,
                                       4.701948,
                                       1.938396,
                                       3.295580,
                                       -1.122583,
                                       3.541429,
                                       3.684907,
                                       1.776786,
                                       -2.282374,
                                       4.339850,
                                       3.168835,
                                       3.134980};

        std::vector<State*> states = debug::doubleVec2StateVec(numbers, this->model_);
        belief1 = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());
    }

    virtual ~PyConversionTest() {
        if (model_ != nullptr)
            delete model_;
        if (rand_ != nullptr)
            delete rand_;

        if (belief1 != nullptr)
            delete belief1;
    }

    py::array_t<double> testPointConv() {
        State* s = this->model_->allocateState();
        s->set(10.0, 0);
        return point2NpArray(*s);
    }

    py::array_t<double> testActionConv() {
        Action act = static_cast<Action>(cld::CLDAction::NEG3);
        return action2NpArray(act, this->model_);
    }

    py::array_t<double> testValueConv() {
        IpftValue val(20, 1);
        return value2NpArray(val);
    }

    PyParticleBelief testParticleBeliefConv() {
        return PyParticleBelief::createPyParticleBelief(*belief1);
    }

    std::shared_ptr<PyVNode> testTreeConv() {
        // create C++ tree
        // create root
        std::vector<double> numbers = {4.253895,
                                       3.168835};
        std::vector<State*> states = debug::doubleVec2StateVec(numbers, this->model_);
        ParticleBelief* b = new ParticleBelief(states, false, this->model_, this->rand_, new NoReinvigoration());
        VNode* root = new VNode(this->model_, nullptr, nullptr, b, 0);

        for (Action a = 0; a < this->model_->numActions(); a++) {
            QNode* qnode = new QNode(this->model_, root, a, 0);  // same level as parent vnode
            qnode->setCount(0);
            Value* initVal = new IpftValue(a, 0.0);
            qnode->setValue(initVal);

            root->children().push_back(qnode);
        }
        // convert to Python tree
        auto pyroot = PyVNode::convertTree(root);
        delete root;
        return pyroot;
    }

    bool testNpParticleArrayConv(const py::array_t<double>& particleArr) {
        std::vector<State*> particleSet = npArray2ParticleSet(particleArr, this->model_);
        ParticleBelief* bel = new ParticleBelief(particleSet, false, this->model_, this->rand_, new NoReinvigoration());
        LOG(INFO) << bel->detailedText();
        bool correct = particleSet[0]->get(0) == 5.0 && particleSet[0]->weight_ == 0.5;
        delete bel;
        return correct;
    }
};

PYBIND11_MODULE(PYTHON_API_MODULE_NAME, m) {
    //* module docstring
    // m.doc() = R"pbdoc(
    //     Pybind11 example plugin
    //     -----------------------
    //     .. currentmodule:: cmake_example
    //     .. autosummary::
    //        :toctree: _generate
    //        add
    //        subtract
    // )pbdoc";

    //* module functions
    // m.def("add", &add, R"pbdoc(
    //     Add two numbers
    //     Some other explanation about the add function.
    // )pbdoc");

    // m.def(
    //     "subtract", [](int i, int j) { return i - j; }, R"pbdoc(
    //     Subtract two numbers
    //     Some other explanation about the subtract function.
    // )pbdoc");

    //* ipft module classes

    /* -------------------------- particle belief class ------------------------- */

    py::class_<PyParticleBelief> pyParticleBelief(m, "PyParticleBelief");
    pyParticleBelief
        .def(py::init<>())
        .def_readwrite("particles_", &PyParticleBelief::particles_)
        .def_readwrite("weights_", &PyParticleBelief::weights_)
        .def_readwrite("post_particles_", &PyParticleBelief::post_particles_)
        .def_readwrite("post_weights_", &PyParticleBelief::post_weights_)
        .def_readwrite("num_particles_", &PyParticleBelief::num_particles_);

    /* --------------------------- search tree classes -------------------------- */

    py::class_<PyNode> pyNode(m, "PyNode");
    pyNode
        .def(py::init<>())
        .def_readwrite("count_", &PyNode::count_)
        .def_readwrite("treelevel_", &PyNode::treelevel_)
        .def_readwrite("values_", &PyNode::values_);

    py::class_<PyVNode, std::shared_ptr<PyVNode> /* <- holder type */> pyVNode(m, "PyVNode", pyNode);
    pyVNode
        .def(py::init<>())
        .def_readwrite("parent_", &PyVNode::parent_)
        .def_readwrite("children_", &PyVNode::children_)
        .def_readwrite("observation_", &PyVNode::observation_)
        .def_readwrite("belief_", &PyVNode::belief_);

    py::class_<PyQNode, std::shared_ptr<PyQNode> /* <- holder type */> pyQNode(m, "PyQNode", pyNode);
    pyQNode
        .def(py::init<>())
        .def_readwrite("parent_", &PyQNode::parent_)
        .def_readwrite("children_", &PyQNode::children_)
        .def_readwrite("action_", &PyQNode::action_);

    /* ---------------------------- pyPlanner classes --------------------------- */

    py::class_<PyPlannerCld> pyPlannerCld(m, "PyPlannerCld");
    pyPlannerCld
        .def(py::init<>())
        .def("resetPlanner", &PyPlannerCld::resetPlanner)
        .def("getSearchTreeOfStep", &PyPlannerCld::getSearchTreeOfStep)
        .def("numSteps", &PyPlannerCld::numSteps)
        .def("runPlanningLoop", &PyPlannerCld::runPlanningLoop);

    /* ----------------------------- pyIpft classes ----------------------------- */

    py::class_<PyIpftCld> pyIpftCld(m, "PyIpftCld");
    pyIpftCld
        .def(py::init<const py::array_t<double>&>())
        .def("reset", &PyIpftCld::reset)
        .def("getSearchTree", &PyIpftCld::getSearchTree)
        .def("search", &PyIpftCld::search)
        .def("getBelief", &PyIpftCld::getBelief);

    /* ------------------------------ test classes ------------------------------ */

    py::class_<PyConversionTest>(m, "PyConversionTest")
        .def(py::init<>())
        .def("testPointConv", &PyConversionTest::testPointConv)
        .def("testActionConv", &PyConversionTest::testActionConv)
        .def("testValueConv", &PyConversionTest::testValueConv)
        .def("testParticleBeliefConv", &PyConversionTest::testParticleBeliefConv)
        .def("testTreeConv", &PyConversionTest::testTreeConv)
        .def("testNpParticleArrayConv", &PyConversionTest::testNpParticleArrayConv);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}

}  // namespace pyadapters
}  // namespace solver_ipft