#include "Module.h"

#include "Array.h"

#include <cstring>

namespace RocketSim::Python
{
PyTypeObject *RotMat::Type = nullptr;

PyMethodDef RotMat::Methods[] = {
    {.ml_name = "as_tuple", .ml_meth = (PyCFunction)&RotMat::AsTuple, .ml_flags = METH_NOARGS, .ml_doc = nullptr},
    {.ml_name = "as_angle", .ml_meth = (PyCFunction)&RotMat::AsAngle, .ml_flags = METH_NOARGS, .ml_doc = nullptr},
    {.ml_name = "as_numpy", .ml_meth = (PyCFunction)&RotMat::AsNumpy, .ml_flags = METH_NOARGS, .ml_doc = nullptr},
    {.ml_name = "__format__", .ml_meth = (PyCFunction)&RotMat::Format, .ml_flags = METH_VARARGS, .ml_doc = nullptr},
    {.ml_name = "__getstate__", .ml_meth = (PyCFunction)&RotMat::Pickle, .ml_flags = METH_NOARGS, .ml_doc = nullptr},
    {.ml_name = "__setstate__", .ml_meth = (PyCFunction)&RotMat::Unpickle, .ml_flags = METH_O, .ml_doc = nullptr},
    {.ml_name = nullptr, .ml_meth = nullptr, .ml_flags = 0, .ml_doc = nullptr},
};

PyGetSetDef RotMat::GetSet[] = {
    GETSET_ENTRY (RotMat, forward),
    GETSET_ENTRY (RotMat, right),
    GETSET_ENTRY (RotMat, up),
    {.name = nullptr, .get = nullptr, .set = nullptr, .doc = nullptr, .closure = nullptr},
};

PyType_Slot RotMat::Slots[] = {
    {Py_tp_new, (void *)&RotMat::New},
    {Py_tp_init, (void *)&RotMat::Init},
    {Py_tp_dealloc, (void *)&RotMat::Dealloc},
    {Py_tp_repr, (void *)&RotMat::Repr},
    {Py_tp_methods, &RotMat::Methods},
    {Py_tp_getset, &RotMat::GetSet},
    {0, nullptr},
};

PyType_Spec RotMat::Spec = {
    .name      = "RocketSim.RotMat",
    .basicsize = sizeof (RotMat),
    .itemsize  = 0,
    .flags     = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_HEAPTYPE,
    .slots     = RotMat::Slots,
};

PyRef<RotMat> RotMat::NewFromRotMat (::RotMat const &mat_) noexcept
{
	auto const self = PyRef<RotMat>::stealObject (RotMat::New (RotMat::Type, nullptr, nullptr));
	if (!self || !InitFromRotMat (self.borrow (), mat_))
		return nullptr;

	return self;
}

bool RotMat::InitFromRotMat (RotMat *const self_, ::RotMat const &mat_) noexcept
{
	auto const forward = Vec::NewFromVec (mat_.forward);
	auto const right   = Vec::NewFromVec (mat_.right);
	auto const up      = Vec::NewFromVec (mat_.up);

	if (!forward || !right || !up)
		return false;

	PyRef<Vec>::assign (self_->forward, forward.borrowObject ());
	PyRef<Vec>::assign (self_->right, right.borrowObject ());
	PyRef<Vec>::assign (self_->up, up.borrowObject ());

	return true;
}

::RotMat RotMat::ToRotMat (RotMat *self_) noexcept
{
	::RotMat mat{};

	mat.forward = Vec::ToVec (self_->forward);
	mat.right   = Vec::ToVec (self_->right);
	mat.up      = Vec::ToVec (self_->up);

	return mat;
}

PyObject *RotMat::New (PyTypeObject *subtype_, PyObject *args_, PyObject *kwds_) noexcept
{
	auto const tp_alloc = (allocfunc)PyType_GetSlot (subtype_, Py_tp_alloc);

	auto self = PyRef<RotMat>::stealObject (tp_alloc (subtype_, 0));
	if (!self)
		return nullptr;

	self->forward = nullptr;
	self->right   = nullptr;
	self->up      = nullptr;

	return self.giftObject ();
}

int RotMat::Init (RotMat *self_, PyObject *args_, PyObject *kwds_) noexcept
{
	::RotMat mat = ::RotMat::GetIdentity ();
	if (PyTuple_Size (args_) == 0 && !kwds_)
	{
		if (!InitFromRotMat (self_, mat))
			return -1;

		return 0;
	}

	if (PyArg_ParseTuple (args_,
	        "fffffffff",
	        &mat.forward.x,
	        &mat.forward.y,
	        &mat.forward.z,
	        &mat.right.x,
	        &mat.right.y,
	        &mat.right.z,
	        &mat.up.x,
	        &mat.up.y,
	        &mat.up.z))
	{
		if (!InitFromRotMat (self_, mat))
			return -1;

		return 0;
	}

	PyErr_Clear ();

	static char forwardKwd[] = "forward";
	static char rightKwd[]   = "right";
	static char upKwd[]      = "up";
	static char *dict[]      = {forwardKwd, rightKwd, upKwd, nullptr};

	PyObject *forward = nullptr; // borrowed references
	PyObject *right   = nullptr;
	PyObject *up      = nullptr;
	if (!PyArg_ParseTupleAndKeywords (
	        args_, kwds_, "O!O!O!", dict, Vec::Type, &forward, Vec::Type, &right, Vec::Type, &up))
		return -1;

	PyRef<Vec>::assign (self_->forward, forward);
	PyRef<Vec>::assign (self_->right, right);
	PyRef<Vec>::assign (self_->up, up);

	return 0;
}

void RotMat::Dealloc (RotMat *self_) noexcept
{
	Py_XDECREF (self_->forward);
	Py_XDECREF (self_->right);
	Py_XDECREF (self_->up);

	auto const tp_free = (freefunc)PyType_GetSlot (Type, Py_tp_free);
	tp_free (self_);
}

PyObject *RotMat::Repr (RotMat *self_) noexcept
{
	auto const tuple = PyObjectRef::steal (AsTuple (self_));
	if (!tuple)
		return nullptr;

	return PyObject_Repr (tuple.borrow ());
}

PyObject *RotMat::Format (RotMat *self_, PyObject *args_) noexcept
{
	auto const forwardString = PyObjectRef::steal (Vec::Format (self_->forward, args_));
	if (!forwardString)
		return nullptr;

	auto const rightString = PyObjectRef::steal (Vec::Format (self_->right, args_));
	if (!rightString)
		return nullptr;

	auto const upString = PyObjectRef::steal (Vec::Format (self_->up, args_));
	if (!upString)
		return nullptr;

	return PyUnicode_FromFormat ("(%S, %S, %S)", forwardString.borrow (), rightString.borrow (), upString.borrow ());
}

PyObject *RotMat::Pickle (RotMat *self_) noexcept
{
	return Py_BuildValue ("{sOsOsO}", "forward", self_->forward, "right", self_->right, "up", self_->up);
}

PyObject *RotMat::Unpickle (RotMat *self_, PyObject *dict_) noexcept
{
	auto const args = PyObjectRef::steal (PyTuple_New (0));
	if (!args)
		return nullptr;

	if (Init (self_, args.borrow (), dict_) != 0)
		return nullptr;

	Py_RETURN_NONE;
}

PyObject *RotMat::Getforward (RotMat *self_, void *) noexcept
{
	return PyRef<Vec>::incRef (self_->forward).giftObject ();
}

int RotMat::Setforward (RotMat *self_, PyObject *value_, void *) noexcept
{
	if (!value_)
	{
		PyErr_SetString (PyExc_TypeError, "can't delete 'forward' attribute of 'RocketSim.RotMat' objects");
		return -1;
	}

	if (!Py_IS_TYPE (value_, Vec::Type))
	{
		PyErr_SetString (PyExc_TypeError, "attribute value type must be RocketSim.Vec");
		return -1;
	}

	PyRef<Vec>::assign (self_->forward, value_);

	return 0;
}

PyObject *RotMat::Getright (RotMat *self_, void *) noexcept
{
	return PyRef<Vec>::incRef (self_->right).giftObject ();
}

int RotMat::Setright (RotMat *self_, PyObject *value_, void *) noexcept
{
	if (!value_)
	{
		PyErr_SetString (PyExc_TypeError, "can't delete 'right' attribute of 'RocketSim.RotMat' objects");
		return -1;
	}

	if (!Py_IS_TYPE (value_, Vec::Type))
	{
		PyErr_SetString (PyExc_TypeError, "attribute value type must be RocketSim.Vec");
		return -1;
	}

	PyRef<Vec>::assign (self_->right, value_);

	return 0;
}

PyObject *RotMat::Getup (RotMat *self_, void *) noexcept
{
	return PyRef<Vec>::incRef (self_->up).giftObject ();
}

int RotMat::Setup (RotMat *self_, PyObject *value_, void *) noexcept
{
	if (!value_)
	{
		PyErr_SetString (PyExc_TypeError, "can't delete 'up' attribute of 'RocketSim.RotMat' objects");
		return -1;
	}

	if (!Py_IS_TYPE (value_, Vec::Type))
	{
		PyErr_SetString (PyExc_TypeError, "attribute value type must be RocketSim.Vec");
		return -1;
	}

	PyRef<Vec>::assign (self_->up, value_);

	return 0;
}

PyObject *RotMat::AsTuple (RotMat *self_) noexcept
{
	return Py_BuildValue ("OOO", self_->forward, self_->right, self_->up);
}

PyObject *RotMat::AsAngle (RotMat *self_) noexcept
{
	return Angle::NewFromAngle (::Angle::FromRotMat (ToRotMat (self_))).giftObject ();
}

PyObject *RotMat::AsNumpy (RotMat *self_) noexcept
{
	auto array = PyArrayRef (3, 3);
	if (!array)
		return nullptr;

	auto const forward = Vec::ToVec (self_->forward);
	auto const right   = Vec::ToVec (self_->right);
	auto const up      = Vec::ToVec (self_->up);

	array (0, 0) = forward.x;
	array (0, 1) = forward.y;
	array (0, 2) = forward.z;
	array (1, 0) = right.x;
	array (1, 1) = right.y;
	array (1, 2) = right.z;
	array (2, 0) = up.x;
	array (2, 1) = up.y;
	array (2, 2) = up.z;

	return array.giftObject ();
}
}
