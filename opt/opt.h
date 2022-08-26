/**
 * @file opt.h
 * @author AMOUSSOU Zinsou Kenneth (www.gitlab.com/azinke)
 * @brief CLI arguments/options parsing
 * @version 0.1
 * @date 2022-08-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef CLI_OPT_H
#define CLI_OPT_H

#ifdef __cplusplus
extern "C" {
#endif

#define OPT_SUCCESS         0

#define EOPT_MEM_ALLOC      (-1)
#define EOPT_NO_ARG         (-2)
#define EOPT_ARG_EXIST      (-3)
#define EOPT_ARG_NO_MATCH   (-4)

/**
 * @brief Type definition for parsing the CLI options
 * 
 */
typedef enum cli_option_type {
  OPT_UNKNOWN,
  OPT_BOOL,
  OPT_SHORT,
  OPT_INT,
  OPT_FLOAT,
  OPT_STR
} cli_option_type_t;

/**
 * @brief Definition of CLI option actions
 *
 * Practical for boolean flags that do not require a value or executing
 * a callback function.
 */
typedef enum {
  OPT_NO_ACTION,
  OPT_STORE_TRUE,
  OPT_CALLBACK,
} cli_option_action_t;


/**
 * @brief Definition of the components of a command line argument
 * 
 */
typedef struct cli_option {
  /**
   * @brief Short CLI argument
   *
   * The expected format: -b
   * Normaly a single dash followed by one letter. But a couple more
   * letters are supported
   */
  const char *args;

  /**
   * @brief Long CLI argument
   *
   * The expected is format: --argname
   * The argument can be provided including the double dashes at
   * the beginning
   */
  const char *argl;

  /**
   * @brief Type of the CLI argument.
   *
   * Will help in parsing the argument
   */
  cli_option_type_t type;

  /**
   * @brief Action to do upon the reception of the argument
   *
   * Useful for boolean flags or initiate a callback function
   */
  cli_option_action_t action;

  /**
   * @brief Callbacl function to call when the argument is received
   * 
   */
  void (*callback)(void);

  /**
   * @brief Help message
   *
   * Reasonable length message describing the meaning of the CLI option
   */
  const char *help;

  /**
   * @brief Default value of the option
   *
   * The void pointer would be casted acording to the type of the argument
   */
  void *default_value;

  /**
   * @brief Value of the parameter after parsing
   *
   * Memory will be dynamically allocated to store the value
   */
  unsigned char *value;

} option_t;


/**
 * @brief CLI Argument data structure
 *
 * The arguments will be organized in a linked list structure
 */
typedef struct cli_arg {
  /**
   * @brief CLI Argument preceding it.
   *
   * The value will be NULL of no argument comes before it 
   */
  void *previous;

  /**
   * @brief Definition of the CLI option
   */
  option_t *opt;

  /**
   * @brief Indicate of the option has been set in CLI
   *
   * If not, the default value is used.
   */
  unsigned char is_set;

  /**
   * @brief Next CLI Argument it's chained to
   * 
   */
  void *next;
} arg_t;


/**
 * @brief CLI root parser
 * 
 */
typedef struct cli_parser {

  /**
   * @brief Name of the CLI parser
   *
   * Can also be considered as the name of the program
   */
  char *name;

  /**
   * @brief Description of the program and what it's meant to do
   * 
   */
  char *description;

  /**
   * @brief Version of the program
   */
  char *version;

  /**
   * @brief Pointer to the first CLI arguments the parser is attached to
   *
   * CLI arguments will be stored in a linked list structure
   */
  arg_t *first_arg;

  /**
   * @brief Pointer to the last CLI arguments the parser is attached to
   */
  arg_t *last_arg;

} parser_t;


/** Create the parser */
parser_t init_parser(const char *name, const char *description);
void free_parser(parser_t *parser);

/** Add new CLI option to the parser */
int add_arg(parser_t *parser, option_t *option);

/** Parse arguments */
int parse(parser_t *parser, int argc, char* argv[]);

/** Read the value of a CLI argument */
void* get_option(parser_t *parser, char *cli_arg);

/** print help */
void print_help(parser_t* parser);

/** Check if a CLI argument matches one of the predefined ones */
int is_arg(char *cli_arg, arg_t *arg);


#ifdef __cplusplus
}
#endif

#endif
