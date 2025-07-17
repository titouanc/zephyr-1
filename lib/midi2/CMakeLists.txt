if(CONFIG_MIDI2_UMP_STREAM_RESPONDER)
  zephyr_include_directories(.)
  zephyr_sources(ump_stream_responder.c)
endif()
