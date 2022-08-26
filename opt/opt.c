/**
 * @file opt.c
 * @author AMOUSSOU Zinsou Kenneth (www.gitlab.com/azinke)
 * @brief CLI arguments/options parsing
 * @version 0.1
 * @date 2022-08-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "opt.h"


/**
 * @brief Create the CLI parser
 *
 * @param name Name of the program
 * @param description Description of the program
 * @return parser_t 
 */
parser_t init_parser(const char* name, const char* description) {
  parser_t parser = {
    .name = (char*) name,
    .description = (char*) description,
  };
  parser.first_arg = NULL;
  parser.last_arg = NULL;
  return parser;
}

/**
 * @brief Add a CLI option to a parser
 * 
 * @param parser Pointer to the target parser
 * @param option Option to be added
 * @return int 
 */
int add_arg(parser_t* parser, option_t *option) {
  // Both short and long option names cannot be NULL
  if ((option->argl == NULL) && (option->args == NULL)) {
    return EOPT_NO_ARG;
  }

  arg_t* argc = malloc(sizeof(arg_t));

  // Couldn't allocate memory
  if (argc == NULL) return EOPT_MEM_ALLOC;

  argc->next = NULL;
  argc->opt = option;
  argc->is_set = 0;

  if (parser->first_arg == NULL) {
    // Add first argument
    argc->previous = NULL;
    parser->first_arg = argc;
    parser->last_arg = argc;
  } else  {
    // Add the new argument at the end of the list
    argc->previous = parser->last_arg;
    parser->last_arg->next = (void*) argc;
    parser->last_arg = argc;
  }
  return OPT_SUCCESS;
}

/**
 * @brief Free the resources allocated to a parser
 * 
 * @param parser 
 * @return void 
 */
void free_parser(parser_t *parser) {
  while(parser->last_arg != NULL) {
    arg_t *previous = (arg_t*) parser->last_arg->previous;
    free(parser->last_arg->opt->value);
    free(parser->last_arg);
    parser->last_arg = previous;
  }
}


/**
 * @brief Print the help related to a parser
 * 
 * @param parser Pointer to the parser
 */
void print_help(parser_t* parser) {
  printf("usage: %s", parser->name);
  arg_t *argc = parser->first_arg;
  while(argc != NULL) {
    if (argc->opt->args != NULL) printf(" [%s]", argc->opt->args);
    else printf(" [%s]", argc->opt->argl);
    argc = (arg_t*)argc->next;
  }

  printf("\n\n%s\n\n", parser->description);
  printf("options:\n");

  argc = parser->first_arg;
  const int buffer_size = 32;
  char buf[buffer_size];
  while(argc != NULL) {
    memset(buf, '\0', buffer_size);
    if (argc->opt->args != NULL){
      strcpy(buf, argc->opt->args);
      strcat(buf, ", ");
    }
    if (argc->opt->argl != NULL) strcat(buf, argc->opt->argl);
    printf("    %-30s ", buf);
    printf("%s \n", argc->opt->help);
    argc = (arg_t*)argc->next;
  }
  printf("\n");
}


/**
 * @brief Parse the CLI arguments
 *
 * @param parser Parser
 * @param argc Argument counts
 * @param argv CLI arguments
 * @return int 
 */
int parse(parser_t *parser, int argc, char* argv[]) {
  if (argc <= 1) return 0;
  for (int idx = 1; idx < argc; idx++) {
    arg_t *arg = parser->first_arg;
    while (arg != NULL) {
      if (is_arg(argv[idx], arg) == OPT_SUCCESS) {
        switch (arg->opt->type) {
          case OPT_BOOL: {
            // Set the boolean as "True" when present
            arg->opt->value = (unsigned char*)malloc(sizeof(char));
            *(arg->opt->value) = 1;
            break;
          }
          case OPT_SHORT: {
            arg->opt->value = (unsigned char*)malloc(sizeof(short));
            sscanf(argv[idx+1], "%hi", (short*)arg->opt->value);
            idx++; // skip the next CLI entry
            break;
          }
          case OPT_INT: {
            arg->opt->value = (unsigned char*)malloc(sizeof(int));
            sscanf(argv[idx+1], "%d", (int*)arg->opt->value);
            idx++; // skip the next CLI entry
            break;
          }
          case OPT_FLOAT: {
            arg->opt->value = (unsigned char*)malloc(sizeof(float));
            sscanf(argv[idx+1], "%f", (float*)arg->opt->value);
            idx++; // skip the next CLI entry
            break;
          }
          case OPT_STR: {
            size_t size = strlen(argv[idx+1]);
            arg->opt->value = (unsigned char*)malloc(size);
            strncpy(arg->opt->value, argv[idx+1], size);
            idx++; // skip the next CLI entry
            break;
          }
        }
        if (arg->opt->callback != NULL) {
          arg->opt->callback();
        }
        arg->is_set = 1;
        break; // Exit the while loop
      }
      arg = (arg_t*)arg->next;
    }
  }
}


/**
 * @brief Read the value of the CLI argument
 * 
 * @param parser 
 * @param cli_arg 
 * @return void* 
 */
void* get_option(parser_t *parser, char *cli_arg) {
  arg_t *arg = parser->first_arg;
  while (arg != NULL) {
    if (is_arg(cli_arg, arg) == OPT_SUCCESS) {
      if (arg->is_set) return arg->opt->value;
      return arg->opt->default_value;
    }
    arg = (arg_t*)arg->next;
  }
  return NULL;
}


/**
 * @brief Check if a CLI argument provided match with a predefined option
 *
 * @param cli_arg Tag of the command line argument - Should start with a single
 *                or double dash character
 * @param arg Predefined argument
 * @return int Status.
 *    EOPT_NO_ARG       : Not a CLI argument
 *    EOPT_SUCESS       : Matched found
 *    EOPT_ARG_NO_MATCH : Not matched
 */
int is_arg(char *cli_arg, arg_t *arg) {
  // Number of single or double dash at the begining of a CLI
  // argument
  int ndash = 0;
  if (cli_arg[0] == '-') ndash++;
  if (cli_arg[1] == '-') ndash++;

  int arglen = strlen(cli_arg + ndash);
  int smatch = -1;
  int lmatch = -1;
  if (arg->opt->args != NULL) {
    smatch = strncmp(cli_arg + ndash, arg->opt->args + 1, arglen);
  }
  if (arg->opt->argl != NULL) {
    lmatch = strncmp(cli_arg + ndash, arg->opt->argl + 2, arglen);
  }
  if ((smatch == 0) || (lmatch == 0)) return OPT_SUCCESS;
  return EOPT_ARG_NO_MATCH;
}
