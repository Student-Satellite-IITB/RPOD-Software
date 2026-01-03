#include "apps/vbn/ImageCapture.hpp"
#include <iostream>

static void printResult(const char* name, bool ok) {
    std::cout << name << ": " << (ok ? "OK" : "FAIL") << "\n";
}

static void printStatus(const vbn::ImageCapture& cap, const char* where) {
    std::cout << "  [" << where << "] status= "
              << vbn::ImageCapture::StatusStr(cap.lastStatus())
              << " errno=" << cap.lastErrno()
              << "\n";
}

int main() {
    using namespace vbn;

    std::cout << "=== vbn_imagecapturebuild_test ===\n";

    {
        std::cout << "\n[Test 0] StatusStr() linkage check\n";
        // Make sure the symbol links and returns a non-null string.
        const char* s = ImageCapture::StatusStr(ImageCapture::Status::OK);
        if (!s) {
            std::cout << "FAIL: StatusStr returned null\n";
            return 1;
        }
        std::cout << "StatusStr(OK) = " << s << "\n";
    }


    {
        std::cout << "\n[Test 1] Start() should fail cleanly (no /dev node on WSL).\n";
        ImageCaptureConfig cfg;
        cfg.dev = "/dev/does_not_exist";
        cfg.buffer_count = 0;
        cfg.bit_depth = 10;
        cfg.bit_shift = 99;

        ImageCapture cap(cfg);

        bool ok = cap.Start();
        printResult("Start()", ok);
        printStatus(cap, "Start()");

        std::cout << "Calling Stop() after failed Start() (should be safe).\n";
        cap.Stop();
        std::cout << "Stop() done.\n";
    }

    {
        std::cout << "\n[Test 2] Stop() idempotence.\n";
        ImageCaptureConfig cfg;
        cfg.dev = "/dev/does_not_exist";
        ImageCapture cap(cfg);

        cap.Stop();
        cap.Stop();
        std::cout << "Stop() called twice: OK\n";

        bool ok = cap.Start();
        printResult("Start()", ok);
        printStatus(cap, "Start()");
        cap.Stop();
        std::cout << "Stop() after Start(): OK\n";
    }


    std::cout << "\nvbn_imagecapturebuild_test: PASS\n";
    return 0;
}
