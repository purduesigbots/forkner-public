/**
 * Logging Utility to print debug info to the SD Card or stdout
 */
#ifndef _LOGGING_HPP_
#define _LOGGING_HPP_

class DataLogger {
	public:
	enum class output { SD_CARD, STD_OUT };

	/**
	 * Creates a logging utility for sending data to the onboard SD Card or to the
	 * standard terminal output.
	 *
	 * \param iout
	 *        The output destination for the logged messages
	 */
	DataLogger(output iout);

	/**
	 * Logs a message to the specified output.
	 *
	 * \param fmt
	 *        The format string to print
	 * \param args
	 *        varags to be added to the format string
	 */
	template <typename... P>
	void log(const char* fmt, P... args) {
		if (file) fprintf(file, fmt, args...);
	}

	/**
	 * Closes the chosen output stream.
	 */
	void close() {
		fflush(file);
		fclose(file);
	}

	~DataLogger() {
		close();
	}

	private:
	FILE* file;
	bool initialized = false;
};

extern DataLogger* logger;

#endif
