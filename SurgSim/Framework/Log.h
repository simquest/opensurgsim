// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// The convenience header that provides the entirety of the logging API.
/// \ingroup loggingAPI
/// \sa loggingAPI

#ifndef SURGSIM_FRAMEWORK_LOG_H
#define SURGSIM_FRAMEWORK_LOG_H

/// \defgroup loggingAPI Logging API
/// The logging API used by OpenSurgSim code.
/// \sa assertAPI
/// @{

/// \defgroup logInternals Internal logging helpers
/// Not meant for public consumption.

#include "SurgSim/Framework/Logger.h"
#include "SurgSim/Framework/LoggerManager.h"
#include "SurgSim/Framework/LogMessage.h"
#include "SurgSim/Framework/LogOutput.h"
#include "SurgSim/Framework/LogMacros.h"
#include "SurgSim/Framework/Assert.h"

/// @}

#endif  // SURGSIM_FRAMEWORK_LOG_H
