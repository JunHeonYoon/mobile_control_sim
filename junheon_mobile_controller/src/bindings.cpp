#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <Eigen/Dense>
#include "junheon_mobile_controller/DifferentialWheel.h"
#include "junheon_mobile_controller/MecanumWheel.h"
#include "junheon_mobile_controller/CasterWheel.h"

namespace bp = boost::python;
using namespace JunheonMobileController;


// Converter for std::vector<std::string>
struct VectorString_to_python
{
    static PyObject* convert(const std::vector<std::string>& vec)
    {
        boost::python::list py_list;
        for (const auto& str : vec)
        {
            py_list.append(str);
        }   
        return bp::incref(py_list.ptr());
    }
};

struct VectorString_from_python
{
    VectorString_from_python()
    {
        bp::converter::registry::push_back(&convertible, &construct, boost::python::type_id<std::vector<std::string>>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        if (!PySequence_Check(obj_ptr)) return nullptr;
        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, bp::converter::rvalue_from_python_stage1_data* data)
    {
        void* storage = ((bp::converter::rvalue_from_python_storage<std::vector<std::string>>*)data)->storage.bytes;
        new (storage) std::vector<std::string>();
        std::vector<std::string>& vec = *(std::vector<std::string>*)(storage);

        int len = PySequence_Size(obj_ptr);
        if (len < 0) bp::throw_error_already_set();
        vec.reserve(len);

        for (int i = 0; i < len; ++i)
        {
            vec.push_back(bp::extract<std::string>(PySequence_GetItem(obj_ptr, i)));
        }

        data->convertible = storage;
    }
};

BOOST_PYTHON_MODULE(junheon_mobile_controller_wrapper_cpp) 
{
    eigenpy::enableEigenPy();
    bp::to_python_converter<std::vector<std::string>, VectorString_to_python>();
    VectorString_from_python();
    
    // Bind DifferentialWheel class
    bp::class_<DifferentialWheel>("DifferentialWheel", bp::init<std::vector<std::string>, double, double, double, double, double, double, double>())
        .def("IK", &DifferentialWheel::IK)
        .def("FK", &DifferentialWheel::FK)
        .def("VelocityCommand", &DifferentialWheel::VelocityCommand);

    // Bind MecanumWheel class
    bp::class_<MecanumWheel>("MecanumWheel", bp::init<std::vector<std::string>, double, double, double, double, double, double, double, double>())
        .def("IK", &MecanumWheel::IK)
        .def("FK", &MecanumWheel::FK)
        .def("VelocityCommand", &MecanumWheel::VelocityCommand);

    // Bind CaesterWheel class
    bp::class_<CasterWheel>("CasterWheel", bp::init<std::vector<std::string>, double, double, double, double, double, double, double, double, double>())
        .def("IK", &CasterWheel::IK)
        .def("FK", &CasterWheel::FK)
        .def("VelocityCommand", &CasterWheel::VelocityCommand);
}
