/* Force linker to include SIMD symbols that are otherwise optimized out */
extern const int jpeg_nbits_table[];

void __attribute__((used)) force_link_simd_symbols(void) {
  /* Reference the symbol to prevent linker from removing it */
  volatile const int* p = jpeg_nbits_table;
  (void)p;
}
