/******************************************************************************/
/*!
\file   SEH.h
\author Braxton DeHate
\date   2/1/2019
\brief
  This file contains the implementation for the structured exception handler.
  To run your system with support for windows minidump logging, simply include
  this file with your project, and initiate your code calling
  trySEH(fn, params...), or if it is a member function, use
  trySEH(instance, memberFn, params...) instead. In the event of a structured
  execption, it will create a file called MINI_DUMP.dmp in the executable's
  root directory which can be run using Visual Studio, which combined with
  symbols and source code can allow you to resume from the point of exception,
  even if the exception happened on a computer without a debugger.

*/
/******************************************************************************/

#ifndef SEH_H
#define SEH_H

/* Includes */
#include <windows.h>
#include <Dbghelp.h>

/* Defines */
#define EMERGENCY_BYTES 16384UL

LONG WINAPI Handler(EXCEPTION_POINTERS* pException);

/******************************************************************************/
/*!
    Calls a non-member function using the structured exception handler.

  \tparam Ret Return type of the function.
  \tparam Args Variadic set of arguments needed by the function.

  \param fn
    The function to call.
  \param arguments
    Variadic set of values to pass into the function.
*/
/******************************************************************************/
template <typename Ret, typename... Args>
Ret trySEH(Ret(*fn)(Args...), Args... arguments)
{
  /* Set extra space in the stack for this thread. In the case of a stack
    overflow, it will allow us enough space to run the handler and dump error
	data. */
  unsigned long byte_count = EMERGENCY_BYTES;
  SetThreadStackGuarantee(&byte_count);

  /* Attempt the function, ready to catch a structured exception. */
  __try
  {
    return fn(arguments...);
  }
  __except (Handler(GetExceptionInformation())) {}
}

/******************************************************************************/
/*!
    Calls a member function using the structured exception handler.

  \tparam R Return type of the function.
  \tparam ClassT Type of the class the function belongs to.
  \tparam Args Variadic set of arguments needed by the function.

  \param instance
    Pointer to a class containing fn.
  \param fn
    The function to call.
  \param arguments
    Variadic set of values to pass into the function.
*/
/******************************************************************************/
template <typename Ret, typename ClassT, typename... Args>
Ret trySEH(ClassT* instance, Ret((ClassT::*fn))(Args...), Args... arguments)
{
  /* Set extra space in the stack for this thread. In the case of a stack
    overflow, it will allow us enough space to run the handler and dump error
	data. */
  unsigned long byte_count = EMERGENCY_BYTES;
  SetThreadStackGuarantee(&byte_count);

  /* Attempt the function, ready to catch a structured exception. */
  __try
  {
    return (instance->*fn)(arguments...);
  }
  __except (Handler(GetExceptionInformation())) {}
}

/******************************************************************************/
/*!
    This is the handler function used by the exception handler system in
	the case of a thrown structured exception.
	
  \param LONG
    Return code which will denote to windows that the exception has not been
	resolved and that it should continue previous behavior.
  \param pException
    Pointer to the windows exception object from which we can pull information.
*/
/******************************************************************************/
LONG WINAPI Handler(EXCEPTION_POINTERS* pException) {
  /* Create a file handle for writing the dump to. */
  HANDLE dumpFile = CreateFile(L"MINI_DUMP.dmp",
    GENERIC_WRITE,
    NULL,
    NULL,
    CREATE_ALWAYS,
    NULL,
    NULL);

  /* Generate an info structure containing necessary information for
    windows to create the minidump. */
  MINIDUMP_EXCEPTION_INFORMATION ExceptionParam = {
    GetCurrentThreadId(),
    pException,
    FALSE
  };

  /* Generate the minidump and write it to the created file. */
  MiniDumpWriteDump(
    GetCurrentProcess(),
    GetCurrentProcessId(),
    dumpFile,
    MiniDumpNormal,
    &ExceptionParam,
    NULL,
    NULL);

  /* Always close the file. */
  CloseHandle(dumpFile);

  /* Returning this will cause the exception resolution to continue.
    This will make it so that we still get the exception popup in VS. */
  return EXCEPTION_CONTINUE_SEARCH;
}

#endif