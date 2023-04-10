#include "Module.h"

#include "Array.h"

#include <cstring>

namespace RocketSim::Python
{
PyTypeObject *Angle::Type = nullptr;

PyMemberDef Angle::Members[] = {
    {.name      = "yaw",
        .type   = T_FLOAT,
        .offset = offsetof (Angle, angle) + offsetof (::Angle, yaw),
        .flags  = 0,
        .doc    = "yaw"},
    {.name      = "pitch",
        .type   = T_FLOAT,
        .offset = offsetof (Angle, angle) + offsetof (::Angle, pitch),
        .flags  = 0,
        .doc    = "pitch"},
    {.name      = "roll",
        .type   = T_FLOAT,
        .offset = offsetof (Angle, angle) + offsetof (::Angle, roll),
        .flags  = 0,
        .doc    = "roll"},
    {.name = nullptr, .type = 0, .offset = 0, .flags = 0, .doc = nullptr},
};

PyMethodDef Angle::Methods[] = {
    {.ml_name = "as_tuple", .ml_meth = (PyCFunction)&Angle::AsTuple, .ml_flags = METH_NOARGS, .ml_doc = nullptr},
    {.ml_name = "as_rot_mat", .ml_meth = (PyCFunction)&Angle::AsRotMat, .ml_flags = METH_NOARGS, .ml_doc = nullptr},
    {.ml_name = "as_numpy", .ml_meth = (PyCFunction)&Angle::AsNumpy, .ml_flags = METH_NOARGS, .ml_doc = nullptr},
    {.ml_name = "__format__", .ml_meth = (PyCFunction)&Angle::Format, .ml_flags = METH_VARARGS, .ml_doc = nullptr},
    {.ml_name = "__getstate__", .ml_meth = (PyCFunction)&Vec::Pickle, .ml_flags = METH_NOARGS, .ml_doc = nullptr},
    {.ml_name = "__setstate__", .ml_meth = (PyCFunction)&Vec::Unpickle, .ml_flags = METH_O, .ml_doc = nullptr},
    {.ml_name = nullptr, .ml_meth = nullptr, .ml_flags = 0, .ml_doc = nullptr},
};

PyType_Slot Angle::Slots[] = {
    {Py_tp_new, (void *)&Angle::New},
    {Py_tp_init, (void *)&Angle::Init},
    {Py_tp_dealloc, (void *)&Angle::Dealloc},
    {Py_tp_repr, (void *)&Angle::Repr},
    {Py_tp_members, &Angle::Members},
    {Py_tp_methods, &Angle::Methods},
    {0, nullptr},
};

PyType_Spec Angle::Spec = {
    .name      = "RocketSim.Angle",
    .basicsize = sizeof (Angle),
    .itemsize  = 0,
    .flags     = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_HEAPTYPE,
    .slots     = Angle::Slots,
};

PyRef<Angle> Angle::NewFromAngle (::Angle const &angle_) noexcept
{
	auto const self = PyRef<Angle>::stealObject (Angle::New (Angle::Type, nullptr, nullptr));
	if (!self || !InitFromAngle (self.borrow (), angle_))
		return nullptr;

	return self;
}

bool Angle::InitFromAngle (Angle *const self_, ::Angle const &angle_) noexcept
{
	self_->angle = angle_;
	return true;
}

::Angle Angle::ToAngle (Angle *self_) noexcept
{
	return self_->angle;
}

PyObject *Angle::New (PyTypeObject *subtype_, PyObject *args_, PyObject *kwds_) noexcept
{
	auto const tp_alloc = (allocfunc)PyType_GetSlot (subtype_, Py_tp_alloc);

	auto self = PyRef<Angle>::stealObject (tp_alloc (subtype_, 0));
	if (!self)
		return nullptr;

	new (&self->angle)::Angle{};

	return self.giftObject ();
}

int Angle::Init (Angle *self_, PyObject *args_, PyObject *kwds_) noexcept
{
	static char yawKwd[]   = "yaw";
	static char pitchKwd[] = "pitch";
	static char rollKwd[]  = "roll";
	static char *dict[]    = {yawKwd, pitchKwd, rollKwd, nullptr};

	::Angle angle{};
	if (!PyArg_ParseTupleAndKeywords (args_, kwds_, "|fff", dict, &angle.yaw, &angle.pitch, &angle.roll))
		return -1;

	if (!InitFromAngle (self_, angle))
		return -1;

	return 0;
}

void Angle::Dealloc (Angle *self_) noexcept
{
	self_->angle.~Angle ();

	auto const tp_free = (freefunc)PyType_GetSlot (Type, Py_tp_free);
	tp_free (self_);
}

PyObject *Angle::Repr (Angle *self_) noexcept
{
	auto const tuple = PyObjectRef::steal (AsTuple (self_));
	if (!tuple)
		return nullptr;

	return PyObject_Repr (tuple.borrow ());
}

PyObject *Angle::Format (Angle *self_, PyObject *args_) noexcept
{
	auto format = PyObject_GetAttrString (reinterpret_cast<PyObject *> (&PyFloat_Type), "__format__");
	if (!format || !PyCallable_Check (format))
		return nullptr;

	PyObject *spec; // borrowed reference
	if (!PyArg_ParseTuple (args_, "O!", &PyUnicode_Type, &spec))
		return nullptr;

	auto const applyFormat = [&] (float x_) -> PyObjectRef {
		auto const value = PyObjectRef::steal (PyFloat_FromDouble (x_));
		if (!value)
			return nullptr;

		auto const formatArgs = PyObjectRef::steal (Py_BuildValue ("OO", value.borrow (), spec));
		if (!formatArgs)
			return nullptr;

		return PyObjectRef::steal (PyObject_Call (format, formatArgs.borrow (), nullptr));
	};

	auto const yaw   = applyFormat (self_->angle.yaw);
	auto const pitch = applyFormat (self_->angle.pitch);
	auto const roll  = applyFormat (self_->angle.roll);
	if (!yaw || !pitch || !roll)
		return nullptr;

	return PyUnicode_FromFormat ("(%S, %S, %S)", yaw.borrow (), pitch.borrow (), roll.borrow ());
}

PyObject *Angle::Pickle (Angle *self_) noexcept
{
	return Py_BuildValue ("{sfsfsf}", "yaw", self_->angle.yaw, "pitch", self_->angle.pitch, "roll", self_->angle.roll);
}

PyObject *Angle::Unpickle (Angle *self_, PyObject *dict_) noexcept
{
	if (!Py_IS_TYPE (dict_, &PyDict_Type))
	{
		PyErr_SetString (PyExc_ValueError, "Pickled object is not a dict.");
		return nullptr;
	}

	auto const yaw   = GetItem (dict_, "yaw");
	auto const pitch = GetItem (dict_, "pitch");
	auto const roll  = GetItem (dict_, "roll");

	if ((yaw && !Py_IS_TYPE (yaw.borrow (), &PyFloat_Type)) ||
	    (pitch && !Py_IS_TYPE (pitch.borrow (), &PyFloat_Type)) ||
	    (roll && !Py_IS_TYPE (roll.borrow (), &PyFloat_Type)))
	{
		PyErr_SetString (PyExc_ValueError, "Pickled object is invalid.");
		return nullptr;
	}

	self_->angle.yaw   = static_cast<float> (PyFloat_AsDouble (yaw.borrow ()));
	self_->angle.pitch = static_cast<float> (PyFloat_AsDouble (pitch.borrow ()));
	self_->angle.roll  = static_cast<float> (PyFloat_AsDouble (roll.borrow ()));

	Py_RETURN_NONE;
}

PyObject *Angle::AsTuple (Angle *self_) noexcept
{
	return Py_BuildValue ("fff", self_->angle.yaw, self_->angle.pitch, self_->angle.roll);
}

PyObject *Angle::AsRotMat (Angle *self_) noexcept
{
	return RotMat::NewFromRotMat (self_->angle.ToRotMat ()).giftObject ();
}

PyObject *Angle::AsNumpy (Angle *self_) noexcept
{
	auto array = PyArrayRef (3);
	if (!array)
		return nullptr;

	array (0) = self_->angle.yaw;
	array (1) = self_->angle.pitch;
	array (2) = self_->angle.roll;

	return array.giftObject ();
}
}
