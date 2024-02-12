#include <iostream>

#include <botan/hash.h>
#include <botan/hex.h>

int main(int32_t argc, char** argv)
{
   if (argc != 2){
      std::cout << "Test <message to hash>" << std::endl;
      return -1;
   }

   std::cout << "message to hash: " << argv[1] << std::endl;

   const auto hash1 = Botan::HashFunction::create_or_throw("SHA-256");
   const auto hash2 = Botan::HashFunction::create_or_throw("SHA-384");
   const auto hash3 = Botan::HashFunction::create_or_throw("SHA-3");

   const auto size = strnlen(argv[1], 2048);
   std::vector<uint8_t> buf(2048);
   std::memcpy(buf.data(), argv[1], sizeof(char) * size);

   // update hash computations with read data
   hash1->update(buf.data(), size);
   hash2->update(buf.data(), size);
   hash3->update(buf.data(), size);

   std::cout << "SHA-256: " << Botan::hex_encode(hash1->final()) << std::endl;
   std::cout << "SHA-384: " << Botan::hex_encode(hash2->final()) << std::endl;
   std::cout << "SHA-3: " << Botan::hex_encode(hash3->final()) << std::endl;

   return 0;
}