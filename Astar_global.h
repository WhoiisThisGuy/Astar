#ifndef ASTAR_GLOBAL_H
#define ASTAR_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(ASTAR_LIBRARY)
#  define ASTAR_EXPORT Q_DECL_EXPORT
#else
#  define ASTAR_EXPORT Q_DECL_IMPORT
#endif

#endif // ASTAR_GLOBAL_H
