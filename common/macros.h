#pragma once

// A macro to disallow the copy constructor and operator= functions 
// This should be used in the priavte:declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
TypeName(TypeName&) = delete;              \
void operator=(TypeName) = delete;
