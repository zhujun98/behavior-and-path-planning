#ifndef BEHAVIOR_AND_PATH_PLANNING_LOGGER_HPP
#define BEHAVIOR_AND_PATH_PLANNING_LOGGER_HPP

#if defined(WITH_BOOST)
  #include "boost/log/trivial.hpp"

  #define LOG_TRACE(MESSAGE) \
    BOOST_LOG_TRIVIAL(trace) << MESSAGE

  #define LOG_DEBUG(MESSAGE) \
    BOOST_LOG_TRIVIAL(debug) << MESSAGE

  #define LOG_INFO(MESSAGE) \
    BOOST_LOG_TRIVIAL(info) << MESSAGE

  #define LOG_WARNING(MESSAGE) \
    BOOST_LOG_TRIVIAL(warning) << MESSAGE

  #define LOG_ERROR(MESSAGE) \
    BOOST_LOG_TRIVIAL(error) << MESSAGE

  #define LOG_FATAL(MESSAGE) \
    BOOST_LOG_TRIVIAL(fatal) << MESSAGE
#else
  #define LOG_TRACE(MESSAGE) \
    std::cout << MESSAGE << std::endl;

  #define LOG_DEBUG(MESSAGE) \
    std::cout << MESSAGE << std::endl;

  #define LOG_INFO(MESSAGE) \
    std::cout << MESSAGE << std::endl;

  #define LOG_WARNING(MESSAGE) \
    std::cout << MESSAGE << std::endl;

  #define LOG_ERROR(MESSAGE) \
    std::cout << MESSAGE << std::endl;

  #define LOG_FATAL(MESSAGE) \
    std::cout << MESSAGE << std::endl;
#endif

#endif //BEHAVIOR_AND_PATH_PLANNING_LOGGER_HPP
