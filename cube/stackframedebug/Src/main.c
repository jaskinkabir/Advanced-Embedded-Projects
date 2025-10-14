int debugFunction(int p, int q) {
    // Local copies of parameters
    int a = p; // 0x05
    int b = q; // 0x07
    // Simple operation on the locals
    int c = a + b; 0x0c
    return c;  // breakpoint here (returning the sum)
}

int main(void) {
    int var1 = 5;
    int var2 = 7;
    // Call the debugFunction with two variables
    int result = debugFunction(var1, var2);
    // Infinite loop (program end)
    while (1) {
        // Do nothing, just loop
    }
}
