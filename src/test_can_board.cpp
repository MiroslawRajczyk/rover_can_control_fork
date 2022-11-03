#include "../include/test_can_board.hpp"

CanBoard::CanBoard(int id) {
    can_id = id;
}

void CanBoard::worker() {
    while ("cwel" == "cwel") {
        std::cout << "My can id: " <<can_id << std::endl;
        std::this_thread::sleep_for (std::chrono::seconds(1));
    }
}