/**
 * @author Isaiah Rondeau
 * @date October 5, 2019
 * @brief Jetson Nano GPIO Library 
 *
 * Jetson Nano GPIO library to access input and output fucntions on the pin headers
 */

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "gpio.h"

/**
 * @brief Export control of userspace to file
 * 
 * Export control of userspace to a file to access a pin's properties
 *
 * @param gpio GPIO pin number
 *
 * @return 0 on success, or -1 on failure and errno is set
 */
int gpio_export(unsigned int gpio) {
	
	int fd, len;
	char buf[MAX_BUF];

	fd = gpio_fd_open(gpio, SYSFS_GPIO_EXPORT, O_WRONLY);
	if (fd < 0) {
		return fd;
	}

	len = sprintf(buf, "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;
}

/**
 * @brief Return control of userspace to kernel
 * 
 * Return control of userspace back to the kernel
 *
 * @param gpio GPIO pin number
 *
 * @return 0 on success, or -1 on failure and errno is set
 */
int gpio_unexport(unsigned int gpio) {

	int fd, len;
	char buf[MAX_BUF];

	fd = gpio_fd_open(gpio, SYSFS_GPIO_UNEXPORT, O_WRONLY);
	if (fd < 0) {
		return fd;
	}

	len = sprintf(buf, "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;
}

/**
 * @breif Assign GPIO pin as INPUT/OUTPUT
 *
 * Assign the given GPIO pin to read either input or output
 *
 * @param gpio GPIO pin number
 * @param direction Direction OUTPUT/INPUT to assign pin
 *
 * @return 0 on success, or -1 on failure and errno is set
 */
int gpio_set_dir(unsigned int gpio, unsigned int direction) {
	
	int fd = gpio_fd_open(gpio, SYSFS_GPIO_DIRECTION, O_WRONLY);
	if (fd < 0) {
		return fd;
	}

	if (direction == OUTPUT) {
		write(fd, "out", 4);
	} else {
		write(fd, "in", 3);
	}

	close(fd);

	return 0;
}

/**
 * @brief Set GPIO value to HIGH/LOW
 * 
 * Set the given GPIO pin value to either high or low
 *
 * @param gpio GPIO pin number
 * @param value Pin value to set
 *
 * @return 0 on success, or -1 on failure and errno is set
 */
int gpio_write(unsigned int gpio, unsigned int value) {
	
	int fd = gpio_fd_open(gpio, SYSFS_GPIO_VALUE, O_WRONLY);
	if (fd < 0) {
		return fd;
	}

	if (value == LOW) {
		write(fd, "0", 2);
	} else {
		write(fd, "1", 2);
	}

	close(fd);

	return 0;
}

/**
 * @breif Read GPIO value
 *
 * Read the given GPIO pin value as either high or low
 *
 * @param gpio GPIO pin number
 *
 * @return Value of GPIO pin as HIGH or LOW, or -1 on failure and errno is set
 */
int gpio_read(unsigned int gpio) {
	
	char ch;

	int fd = gpio_fd_open(gpio, SYSFS_GPIO_VALUE, O_RDONLY);
	if (fd < 0) {
		return fd;
	}

	read(fd, &ch, 1);
	close(fd);

	if (ch == '0') {
		return 0;
	} else {
		return 1;
	}
}

/**
 * @breif Set GPIO edge trigger
 *
 * Set the given GPIO pin to change value based on the rising or falling edge
 *
 * @param gpio GPIO pin number
 * @param edge Edge value RISING, FALLING, BOTH, or NONE
 *
 * @return 0 on success, or -1 on failure and errno is set
 */
int gpio_set_edge(unsigned int gpio, char* edge) {
	
	int fd = gpio_fd_open(gpio, SYSFS_GPIO_EDGE, O_WRONLY);
	if (fd < 0) {
		return fd;
	}

	write(fd, edge, strlen(edge) + 1);
	close(fd);

	return 0;
}

/**
 * @brief Set GPIO to read active low
 * 
 * Set the GPIO pin to read values in the active low state
 *
 * @param gpio GPIO pin number
 * @param option Write any number greater than zero to set the logic to active low
 *
 * @return 0 on success, or -1 on failure and errno is set
 */
int gpio_set_active_low(unsigned int gpio, unsigned int option) {
	
	int fd = gpio_fd_open(gpio, SYSFS_GPIO_ACTIVE_LOW, O_WRONLY);
	if (fd < 0) {
		return fd;
	}

	if (option == FALSE) {
		write(fd, "0", 2);
	} else {
		write(fd, "1", 2);
	}

	close(fd);

	return 0;
}

/**
 * @brief Open GPIO file descriptor
 *
 * Open the given GPIO pin's file descriptor
 *
 * @param gpio GPIO pin number
 * @param sysfs_file File to open file descriptor from
 * @param flags File descriptor open() flags
 *
 * @return File descriptor on success, or -1 on failure and errno is set
 */
int gpio_fd_open(unsigned int gpio, char* sysfs_file, int flags) {
	
	char buf[MAX_BUF];

	if (sysfs_file == SYSFS_GPIO_EXPORT || sysfs_file == SYSFS_GPIO_UNEXPORT) {
		snprintf(buf, sizeof(buf), "%s/%s", SYSFS_GPIO_DIR, sysfs_file);
	} else {
		snprintf(buf, sizeof(buf), "%s/gpio%d/%s", SYSFS_GPIO_DIR, gpio, sysfs_file);
	}

	return open(buf, flags);
}
