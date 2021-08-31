#include "logger.h"

std::string to_string(LogType lot) {
  switch (lot) {
  case LogType::STRING:
    return "string";
  case LogType::DOUBLE:
    return "double";
  }
  return "Unrecognized log type";
}

Logger::Logger() {}

template <>
void Logger::log(std::string name, const char* val) {
  logString(name, std::string(val));
}

template <>
void Logger::log(std::string name, std::string val) {
  logString(name, val);
}

template <>
void Logger::log(std::string name, bool val) {
  logString(name, val ? "True" : "False");
}

template <>
void Logger::log(std::string name, double val) {
  logDouble(name, val);
}

template <>
void Logger::log(std::string name, size_t val) {
  logDouble(name, val);
}

template <>
void Logger::log(std::string name, int val) {
  logDouble(name, val);
}

void Logger::logString(std::string name, std::string val) {
  // Try to update an existing field
  for (const std::tuple<std::string, LogType, size_t>& log : logs) {
    if (std::get<0>(log) == name && std::get<1>(log) == LogType::STRING) {
      stringLogs[std::get<2>(log)] = val;
      return;
    }
  }

  logs.push_back(std::make_tuple(name, LogType::STRING, stringLogs.size()));
  stringLogs.push_back(val);
}
void Logger::logDouble(std::string name, double val) {
  // Try to update an existing field
  for (const std::tuple<std::string, LogType, size_t>& log : logs) {
    if (std::get<0>(log) == name && std::get<1>(log) == LogType::DOUBLE) {
      doubleLogs[std::get<2>(log)] = val;
      return;
    }
  }
  logs.push_back(std::make_tuple(name, LogType::DOUBLE, doubleLogs.size()));
  doubleLogs.push_back(val);
}

bool Logger::writeLog(std::string filename) {
  std::ofstream out;

  // std::ios::trunc ensures that we overwrite anything previously in the file
  out.open(filename, std::ios::trunc);
  if (out.is_open()) {
    writeLog(out);
    out.close();
    return true;
  } else {
    return false;
  }
}

void Logger::writeLog(std::ostream& out) {
  out << std::setw(12);
  size_t N = logs.size();
  for (size_t iL = 0; iL + 1 < N; ++iL) {
    out << std::get<0>(logs[iL]) << "\t";
  }
  out << std::get<0>(logs[logs.size() - 1]) << std::endl;

  auto printLog = [&](size_t iL) {
    switch (std::get<1>(logs[iL])) {
    case LogType::STRING:
      out << stringLogs[std::get<2>(logs[iL])];
      break;
    case LogType::DOUBLE:
      out << doubleLogs[std::get<2>(logs[iL])];
      break;
    }
  };

  for (size_t iL = 0; iL + 1 < N; ++iL) {
    printLog(iL);
    out << "\t";
  }
  printLog(logs.size() - 1);
}
