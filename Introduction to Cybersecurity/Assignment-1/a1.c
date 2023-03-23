#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

// Helper function to print environment variables
void printenv(char *envp[]) {
    for (int i = 0; envp[i]; i++)
        printf("[%03d] %p %s\n", i, envp[i], envp[i]);
}

// Function fuzzee - not to be modified
void fuzzee(int argc, char *argv[], char *envp[], char *fmt_input, char *output) {
    // Use the fmt_input format string to write into the output buffer
    sprintf(output, fmt_input);
}

// Main fuzzer function to find and modify the SECRET_ENV variable
void fuzzer(int argc, char *argv[], char *envp[]) {
    char buffer[512];
    char *secret_env_addr = NULL;
    char *temp_envp[2];

    // Search for the SECRET_ENV variable in the environment variables
    for (int i = 0; envp[i]; i++) {
        if (strncmp(envp[i], "SECRET_ENV=", 11) == 0) {
            secret_env_addr = envp[i] + 11; // Skip the "SECRET_ENV=" part.
            temp_envp[0] = envp[i];
            temp_envp[1] = NULL;
            break;
        }
    }

    if (secret_env_addr) {
        // ----- BONUS PART: Craft the payload and modify the SECRET_ENV variable -----
        snprintf(buffer, sizeof(buffer), "hacked");
        memcpy(secret_env_addr, buffer, strlen(buffer) + 1);

        // Call fuzzee() with the crafted payload
        char output[512];
        fuzzee(argc, argv, temp_envp, buffer, output);

        // Check if the attack was successful
        if (strcmp(secret_env_addr, "hacked") == 0) {
            printf("The SECRET_ENV variable has been successfully modified to 'hacked'.\n");
        } else {
            printf("The attack failed. The SECRET_ENV variable is still: %s\n", secret_env_addr);
        }
        // ----- END OF BONUS PART -----
    } else {
        printf("SECRET_ENV not found in the environment variables.\n");
    }
}

// Main function - not to be modified
int main(int argc, char *argv[], char *envp[]) {
    // Print the initial environment variables
    printenv(envp);
    printf("--- --- ---\n");

    // Run the fuzzer
    fuzzer(argc, argv, envp);
    printf("--- --- ---\n");

    // Print the updated environment variables
    printenv(envp);
}

