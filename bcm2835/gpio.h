#include <Python.h>
#include "_bcm2835.h"

static PyMethodDef module_methods[] = {
    {"system",  spam_system, METH_VARARGS,
     "Execute a shell command."},

    {NULL, NULL, 0, NULL}        /* Sentinel - Dummy, i think so*/
};


PyMODINIT_FUNC initgpio(void);