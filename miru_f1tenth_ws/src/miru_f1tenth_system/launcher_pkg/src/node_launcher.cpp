#include <cstdio>
#include <cstdlib>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  
  printf("hello world miru_challenge package\n");
  printf("Launching gap_follow reactive_node_node...\n");
  
  // 시스템 명령어 실행
  int result = std::system("ros2 run gap_follow reactive_node");
  
  if (result == 0) {
    printf("Successfully launched disparity_node\n");
  } else {
    printf("Failed to launch disparity_node. Error code: %d\n", result);
  }
  
  return 0;
}
