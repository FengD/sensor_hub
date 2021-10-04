// Copyright (C) 2021 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: common

#pragma once

#include <iostream>
#include <cyber/cyber.h>
#include "common/concurrent_object_pool.h"
#include "common/error_code.h"
#include "common/factory.h"
#include "common/for_each.h"
#include "common/macros.h"
#include "common/singleton.h"
#include "common/thread.h"
#include "common/thread_safe_queue.h"
#include "common/io/file.h"

#define MAX_THREAD_NAME_LENGTH 21