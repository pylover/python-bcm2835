

PyMODINIT_FUNC initgpio(void)
{
    PyObject *m;
    m = Py_InitModule("gpio", module_methods);
    if (m == NULL)
        return;
}