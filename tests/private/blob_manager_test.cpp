#include "blob_manager.hpp" // Replace with the correct path to your header
#include <doctest/doctest.h>

TEST_CASE("Blob registration")
{
  free_blob_manager_t manager;

  SUBCASE("Register a single blob")
  {
    auto offset = manager.register_blob({10});
    REQUIRE(offset.data == 0);
  }

  SUBCASE("Register multiple blobs")
  {
    auto offset1 = manager.register_blob({10});
    auto offset2 = manager.register_blob({20});
    REQUIRE(offset1.data == 0);
    REQUIRE(offset2.data == 10);
  }
}

TEST_CASE("Blob unregistration and reuse of space")
{
  free_blob_manager_t manager;
  auto offset1 = manager.register_blob({10});
  auto offset2 = manager.register_blob({20});

  SUBCASE("Unregister a blob and check free space")
  {
    REQUIRE(manager.unregister_blob(offset1, {10}) == true);
    auto offset3 = manager.register_blob({5});
    REQUIRE(offset3.data == 0); // Reuse the free space
  }

  SUBCASE("Unregister multiple blobs and check merging of free space")
  {
    REQUIRE(manager.unregister_blob(offset1, {10}) == true);
    REQUIRE(manager.unregister_blob(offset2, {20}) == true);
    auto offset3 = manager.register_blob({25});
    REQUIRE(offset3.data == 0); // Merged free space should allow a blob of size 25
  }
}

TEST_CASE("Registering blobs with no available space")
{
  free_blob_manager_t manager;

  SUBCASE("Register a blob larger than any free section")
  {
    auto offset1 = manager.register_blob({10});
    auto offset2 = manager.register_blob({20});
    REQUIRE(manager.unregister_blob({0}, {10}) == true); // Unregister first blob to create a gap
    auto offset = manager.register_blob({15});
    REQUIRE(offset.data == 30); // Offset should be at the end of the first blob
  }
}

TEST_CASE("Incorrect unregistration")
{
  free_blob_manager_t manager;

  SUBCASE("Unregister a blob that was not registered")
  {
    REQUIRE(manager.unregister_blob({100}, {10}) == false); // Trying to unregister a non-existent blob
  }
}

TEST_CASE("Unregister blob with end outside file size")
{
  free_blob_manager_t manager;
  auto offset = manager.register_blob({50});
  REQUIRE(offset.data == 0); // Assuming initial file size is zero

  SUBCASE("Unregister blob that ends beyond the file size")
  {
    // Assuming current file size is now 50 due to previous registration
    REQUIRE_FALSE(manager.unregister_blob({40}, {20})); // This should fail as it would end at offset 60
  }
}

TEST_CASE("Unregister blob that starts in free space")
{
  free_blob_manager_t manager;
  auto offset1 = manager.register_blob({10});
  auto offset2 = manager.register_blob({20});
  REQUIRE(offset1.data == 0);
  REQUIRE(offset2.data == 10);

  // Create a free section
  bool unregistered = manager.unregister_blob(offset1, {10});
  REQUIRE(unregistered);

  SUBCASE("Unregister blob that starts in a free section")
  {
    REQUIRE_FALSE(manager.unregister_blob({5}, {10})); // This should fail as it starts within a free section
  }
}

TEST_CASE("Unregister blob that ends in free space")
{
  free_blob_manager_t manager;
  auto offset1 = manager.register_blob({10});
  auto offset2 = manager.register_blob({20});
  auto offset3 = manager.register_blob({30});
  REQUIRE(offset1.data == 0);
  REQUIRE(offset2.data == 10);
  REQUIRE(offset3.data == 30);

  // Create a free section
  bool unregistered = manager.unregister_blob(offset2, {20});
  REQUIRE(unregistered);

  SUBCASE("Unregister blob that ends in a free section")
  {
    REQUIRE_FALSE(manager.unregister_blob({5}, {15})); // This should fail as it ends within a free section
  }
}

TEST_CASE("Unregister blob that starts and ends in free space")
{
  free_blob_manager_t manager;
  auto offset1 = manager.register_blob({10});
  auto offset2 = manager.register_blob({20});
  REQUIRE(offset1.data == 0);
  REQUIRE(offset2.data == 10);

  bool unregistered = manager.unregister_blob(offset1, {10});
  REQUIRE(unregistered);

  SUBCASE("Unregister blob that encompasses a free section")
  {
    REQUIRE_FALSE(manager.unregister_blob({5}, {20})); // This should fail as it starts and ends within a free section
  }
}

// Additional tests can be written to cover more edge cases and scenarios.

TEST_CASE("Merging free spaces")
{
  free_blob_manager_t manager;

  SUBCASE("Unregister and merge adjacent blobs")
  {
    auto offset1 = manager.register_blob({10});
    auto offset2 = manager.register_blob({20});
    auto offset3 = manager.register_blob({20});
    REQUIRE(manager.unregister_blob(offset1, {10}) == true);
    REQUIRE(manager.unregister_blob(offset2, {20}) == true);
    REQUIRE(manager.get_free_sections_count() == 1);
    REQUIRE(manager.get_free_section(0,0).size.data == 30); // Assuming that manager can retrieve a free section by index
  }
}
TEST_CASE("File size decreases when the last blob is unregistered")
{
  free_blob_manager_t manager;
  auto initial_size = manager.get_file_size(); // Assuming this method exists

  SUBCASE("Register and unregister blobs at the end")
  {
    auto offset1 = manager.register_blob({10});
    auto offset2 = manager.register_blob({20});
    REQUIRE(offset1.data == initial_size.data);      // Offset should be at the end of the file
    REQUIRE(offset2.data == initial_size.data + 10); // Offset should be at the end of the file after offset1

    // File size should increase by 30 (size of both blobs)
    REQUIRE(manager.get_file_size().data == initial_size.data + 30);

    // Unregister the last blob
    REQUIRE(manager.unregister_blob(offset2, {20}));

    // File size should decrease by the size of the last blob
    REQUIRE(manager.get_file_size().data == initial_size.data + 10);

    // Unregister the second last blob, which is now the last blob
    REQUIRE(manager.unregister_blob(offset1, {10}));

    // File size should decrease by the size of the now last blob
    REQUIRE(manager.get_file_size().data == initial_size.data);
  }
}

TEST_CASE("Unregister blob should merge with adjacent free sections")
{
  free_blob_manager_t manager;

  // Register and then unregister multiple blobs to create non-contiguous free sections.
  auto offset1 = manager.register_blob({10});
  auto offset2 = manager.register_blob({20});
  auto offset3 = manager.register_blob({30});
  auto offset4 = manager.register_blob({40});

  // Set up free sections that are not contiguous
  REQUIRE(manager.unregister_blob(offset1, {10}) == true); // Free section at the beginning
  REQUIRE(manager.unregister_blob(offset3, {30}) == true); // Free section in the middle

  // This leaves a used section from offset 10 to 30, surrounded by free sections.

  SUBCASE("Unregister a blob that bridges two free sections")
  {
    // Now unregister the blob that is between two free sections
    REQUIRE(manager.unregister_blob(offset2, {20}) == true);

    REQUIRE(manager.get_free_sections_count() == 1);
    REQUIRE(manager.get_free_section(0, 0).size.data == 60);
  }
}

TEST_CASE("Unregistering blob overlapping with free section should fail")
{
  free_blob_manager_t manager;

  // Register blobs to set up the test.
  auto offset1 = manager.register_blob({10});
  auto offset2 = manager.register_blob({20});
  auto offset3 = manager.register_blob({30});

  // Unregister the middle blob to create a free section within the file.
  REQUIRE(manager.unregister_blob(offset2, {20}) == true); // Free section in the middle

  // Confirm the file size has not shrunk.
  auto current_file_size = manager.get_file_size();
  REQUIRE(current_file_size.data == offset1.data + 10 + 20 + 30);

  // Try to unregister a blob that overlaps with the free section.
  SUBCASE("Unregister blob that overlaps with the beginning of a free section")
  {
    REQUIRE_FALSE(manager.unregister_blob({15}, {10}));
  }

  SUBCASE("Unregister blob that overlaps with the end of a free section")
  {
    REQUIRE_FALSE(manager.unregister_blob({5}, {20}));
  }

  SUBCASE("Unregister blob that is completely within a free section")
  {
    REQUIRE_FALSE(manager.unregister_blob({12}, {5}));
  }

  SUBCASE("Unregister blob that spans across the entire free section")
  {
    REQUIRE_FALSE(manager.unregister_blob({5}, {30}));
  }

  REQUIRE(manager.get_file_size().data == current_file_size.data);
}

TEST_CASE("Register blobs that fit within a single page") {
    free_blob_manager_t manager;

    // Assuming FREE_BLOB_MANAGER_PAGE_SIZE is large enough for these blobs.
    REQUIRE(manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 4}).data >= 0);
    REQUIRE(manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 4}).data >= 0);
    REQUIRE(manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 4}).data >= 0);
    REQUIRE(manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 4}).data >= 0);

    REQUIRE(manager.get_free_sections_count() == 0); // No unregistration has happened yet.
}

TEST_CASE("Register and unregister blobs causing overflow on pages") {
    free_blob_manager_t manager;

    auto offset1 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE - 1});
    auto offset2 = manager.register_blob({2}); // This should cause an overflow to the next page

    REQUIRE(manager.unregister_blob(offset1, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE - 1}) == true);
    REQUIRE(manager.unregister_blob(offset2, {2}) == true);

    REQUIRE(manager.get_free_sections_count() == 0); // There should be two free sections, one on each page
}

TEST_CASE("Unregister blobs to create a completely free page") {
    free_blob_manager_t manager;

    auto offset1 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2});
    auto offset2 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2});

    REQUIRE(manager.unregister_blob(offset1, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2}) == true);
    REQUIRE(manager.unregister_blob(offset2, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2}) == true);

    REQUIRE(manager.get_free_sections_count() == 0); 
}

TEST_CASE("Unregister blobs that span multiple pages") {
    free_blob_manager_t manager;

    auto offset1 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2});
    auto offset2 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE}); // This blob spans two pages

    REQUIRE(manager.unregister_blob(offset1, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2}) == true);
    REQUIRE(manager.unregister_blob(offset2, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE}) == true);

    REQUIRE(manager.get_free_sections_count() == 0);
}

TEST_CASE("Edge case: Unregister a blob at the boundary of two pages") {
    free_blob_manager_t manager;

    auto offset = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE - 1});
    auto offset2 = manager.register_blob({1});
    auto offset3 = manager.register_blob({1});
    auto offset4 = manager.register_blob({1});
    auto offset5 = manager.register_blob({1});

    REQUIRE(manager.unregister_blob(offset, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE - 1}) == true);
    REQUIRE(manager.get_free_sections_count() == 1);
    REQUIRE(manager.unregister_blob(offset2, {1}) == true);
    REQUIRE(manager.get_free_sections_count() == 1);
    REQUIRE(manager.unregister_blob(offset3, {1}) == true);
    REQUIRE(manager.get_free_sections_count() == 1);
    REQUIRE(manager.unregister_blob(offset4, {1}) == true);
    REQUIRE(manager.get_free_sections_count() == 1);
    REQUIRE(manager.unregister_blob(offset5, {1}) == true);
    REQUIRE(manager.get_free_sections_count() == 0);
}

TEST_CASE("Registering blob that spans multiple pages")
{
    free_blob_manager_t manager;

    SUBCASE("Register blob spanning across pages")
    {
    auto large_blob_size = uint32_t(free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 1.5);
    auto offset = manager.register_blob({large_blob_size});
    REQUIRE(offset.page() == 0);                                          // Starts on the first page
    REQUIRE((offset.data + large_blob_size) > free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE); // Spans to the next page
    }
}

TEST_CASE("Unregistering blob that spans multiple pages")
{
    free_blob_manager_t manager;
    auto large_blob_size = uint32_t(free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 1.5);
    auto offset = manager.register_blob({large_blob_size});

    SUBCASE("Unregister blob spanning across pages")
    {
    REQUIRE(manager.unregister_blob(offset, {large_blob_size}) == true);
    auto free_section_count = manager.get_free_sections_count();
    REQUIRE(free_section_count == 0); // Should have two free sections, one on each page
    }
}

TEST_CASE("Page removal upon blob unregistration") {
  free_blob_manager_t manager;

  SUBCASE("Unregister all blobs on a page") {
    auto offset = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE});
    REQUIRE(manager.unregister_blob(offset, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE}) == true);
    REQUIRE(manager.get_pages_count() == 0); // The page should be removed
  }
}

TEST_CASE("Page addition upon blob registration") {
  free_blob_manager_t manager;

  SUBCASE("Register blob causing new page to be added") {
    auto offset = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE + 2});
    auto unreg = manager.unregister_blob(offset, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE + 1});
    REQUIRE(unreg);
    REQUIRE(manager.get_pages_count() == 2); // Should now track more than one page
  }
}

TEST_CASE("Merging across pages") {
  free_blob_manager_t manager;

  SUBCASE("Unregister blobs to create non-contiguous free space across pages") {
    auto offset1 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE - 1});
    auto offset2 = manager.register_blob({2}); // This will start at the end of page 0 and spill over to page 1
    REQUIRE(manager.unregister_blob(offset1, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE - 1}) == true);
    REQUIRE(manager.unregister_blob(offset2, {2}) == true);
    REQUIRE(manager.get_pages_count() == 0); // Ensure that two pages are tracked separately
  }
}

TEST_CASE("Fragmentation within a page") {
  free_blob_manager_t manager;

  SUBCASE("Create fragmented free space within a page") {
    auto offset1 = manager.register_blob({10});
    auto offset2 = manager.register_blob({10});
    auto offset3 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE - 20});
    REQUIRE(manager.unregister_blob(offset1, {10}) == true);
    REQUIRE(manager.unregister_blob(offset2, {10}) == true);
    auto offset4 = manager.register_blob({20}); // This should fit into the fragmented space
    REQUIRE(offset4.data == offset1.data); // Should occupy the space from the first unregistered blob
  }
}

TEST_CASE("Handling of full pages") {
  free_blob_manager_t manager;

  SUBCASE("Completely fill a page then unregister some blobs") {
    auto offset1 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2});
    auto offset2 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2});
    REQUIRE(manager.unregister_blob(offset1, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2}) == true);
    REQUIRE(manager.unregister_blob(offset2, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2}) == true);
    REQUIRE(manager.get_pages_count() == 0); // The page should have been removed
  }
}

TEST_CASE("Consecutive page filling and freeing") {
  free_blob_manager_t manager;

  SUBCASE("Fill and free multiple pages consecutively") {
    auto offset1 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE});
    auto offset2 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE});
    REQUIRE(manager.unregister_blob(offset2, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE}) == true);
    REQUIRE(manager.unregister_blob(offset1, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE}) == true);
    REQUIRE(manager.get_pages_count() == 0); // Both pages should have been removed
  }
}

TEST_CASE("Sparse free sections on multiple pages") 
{
  free_blob_manager_t manager;

  SUBCASE("Create non-contiguous free sections across multiple pages") {
    auto offset1 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 3});
    auto offset2 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 3});
    auto offset3 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 3});
    REQUIRE(manager.unregister_blob(offset1, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 3}) == true);
    REQUIRE(manager.unregister_blob(offset3, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 3}) == true);
    auto offset4 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 3}); // Should fit into the first free section
    REQUIRE(offset4.data == offset1.data);
  }
}

TEST_CASE("Test reuse of section bigger than page size") 
{
  free_blob_manager_t manager;

  SUBCASE("Create contiguous free sections across multiple pages") {
    auto offset1 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2});
    auto offset2 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3});
    auto offset3 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3});
    REQUIRE(manager.unregister_blob(offset2, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3}) == true);
    auto offset4 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 2});
    auto offset5 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 3});
    auto offset6 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3});
    
    REQUIRE(offset4.data == offset2.data);
    REQUIRE(manager.unregister_blob(offset3, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3}) == true);
    REQUIRE(manager.get_free_sections_count() == 1);
  }
}

TEST_CASE("Test section merging") 
{
  free_blob_manager_t manager;

  SUBCASE("Create contiguous free sections across multiple pages doesnt ignore allocated space") {
    auto offset1 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE / 2});
    auto offset2 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3});
    auto offset3 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3});
    auto offset4 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3});
    auto offset5 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3});
    auto offset6 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3});

    REQUIRE(manager.unregister_blob(offset2, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3}) == true);
    REQUIRE(manager.unregister_blob(offset3, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3}) == true);
    REQUIRE(manager.unregister_blob(offset5, {free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 3}) == true);
    
    REQUIRE(manager.get_free_sections_count() == 2);

    auto offset7 = manager.register_blob({free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE * 11});
    REQUIRE(offset7.data > offset6.data);
  }
}

// Regression: register_blob must return the freed section's own offset, not the
// page start. Previously a mid-page free section returned offset 0 (page start),
// which is a live blob -> silent overwrite/corruption.
TEST_CASE("Reuse a strictly-middle free section returns its real offset")
{
  free_blob_manager_t manager;
  auto a = manager.register_blob({10}); // @0
  auto b = manager.register_blob({10}); // @10
  auto c = manager.register_blob({10}); // @20
  REQUIRE(a.data == 0);
  REQUIRE(b.data == 10);
  REQUIRE(c.data == 20);

  // Free the MIDDLE blob, leaving a live blob (a@0) before the free section.
  REQUIRE(manager.unregister_blob(b, {10}) == true);

  // Reallocating 10 bytes must reuse the freed slot at offset 10, NOT page start 0.
  auto reused = manager.register_blob({10});
  REQUIRE(reused.data == 10);
}

// Regression: freeing sections within a page in DECREASING offset order must keep
// the per-page free list sorted, so later overlap detection still works and a
// double-free is rejected rather than silently recording overlapping free regions.
TEST_CASE("Free sections stay sorted regardless of free order")
{
  free_blob_manager_t manager;
  auto a = manager.register_blob({10}); // @0
  auto b = manager.register_blob({10}); // @10
  auto c = manager.register_blob({10}); // @20
  auto d = manager.register_blob({10}); // @30
  (void)b;
  (void)d;

  // Free the higher-offset blob first, then a strictly-lower one (non-adjacent).
  REQUIRE(manager.unregister_blob(c, {10}) == true); // free [20,30)
  REQUIRE(manager.unregister_blob(a, {10}) == true); // free [0,10)  (lower offset, appended-late in old code)

  // A double free that overlaps the [20,30) free region must be detected and rejected.
  REQUIRE(manager.unregister_blob({22}, {5}) == false);

  // The two disjoint free regions are still individually reusable.
  auto r0 = manager.register_blob({10});
  auto r1 = manager.register_blob({10});
  REQUIRE(((r0.data == 0 && r1.data == 20) || (r0.data == 20 && r1.data == 0)));
}

// Regression: page-number * PAGE_SIZE must be computed in 64-bit. For files beyond
// ~4 GB (page index >= 41) the old 32-bit product wrapped, corrupting free-section
// offset math. register_blob only tracks offsets, so we can drive it past 4 GB cheaply.
TEST_CASE("Blob manager handles offsets beyond 4 GB without 32-bit overflow")
{
  free_blob_manager_t manager;
  const uint32_t page = free_blob_manager_t::FREE_BLOB_MANAGER_PAGE_SIZE; // 100 MB
  std::vector<free_blob_manager_t::offset_t> offsets;
  // 45 pages ~= 4.5 GB, well past the 4 GB (page 41) wrap point.
  for (int i = 0; i < 45; i++)
    offsets.push_back(manager.register_blob({page}));

  // The 43rd blob starts at byte offset 42 * 100MB = 4,404,019,200 (> 2^32).
  REQUIRE(offsets[42].data == uint64_t(42) * page);

  // Free a blob living well past 4 GB, then reallocate the same size: must reuse
  // exactly that offset (correct 64-bit page arithmetic), not a wrapped-around one.
  auto high = offsets[43];
  REQUIRE(high.data == uint64_t(43) * page);
  REQUIRE(manager.unregister_blob(high, {page}) == true);
  auto reused = manager.register_blob({page});
  REQUIRE(reused.data == high.data);
}

TEST_CASE("free_blob_manager: a spillover allocation must consume the previous page's tail")
{
  // THE SPILLOVER LEAK. register_blob can satisfy a request from the tail of one page plus the head
  // of the next, and returns an offset that starts inside that tail. It then removes the tail from
  // the free list only if the blob spans at least one WHOLE page:
  //
  //     auto spillover_page_count = size.data / FREE_BLOB_MANAGER_PAGE_SIZE;   // 0 for any small blob
  //
  // For anything smaller than a page that is zero, so the tail stays free and is handed out again --
  // on top of the blob that already occupies it. Found as silent data corruption on a 300-scan Ford
  // import: a unit's position blob had been overwritten by a later allocation, and 15 pairs of
  // blobs overlapped in a dataset where every blob should be disjoint.
  using manager_t = free_blob_manager_t;
  constexpr uint64_t page = manager_t::FREE_BLOB_MANAGER_PAGE_SIZE;

  manager_t manager;
  // Grow the file past two pages so there is something to free inside it.
  const auto big = manager.register_blob({uint32_t(page)});
  const auto second = manager.register_blob({uint32_t(page)});
  REQUIRE(big.data == 0);
  REQUIRE(second.data == page);

  // Free the tail of page 0 and the head of page 1, leaving a run that straddles the boundary.
  constexpr uint32_t tail = 4096;
  constexpr uint32_t head = 16384;
  REQUIRE(manager.unregister_blob({page - tail}, {tail}));
  REQUIRE(manager.unregister_blob({page}, {head}));

  // A request bigger than either piece alone can only be met by spanning the boundary.
  constexpr uint32_t spanning = tail + 8192;
  const auto a = manager.register_blob({spanning});
  CHECK(a.data == page - tail);

  // Anything allocated afterwards must not land inside it.
  const auto b = manager.register_blob({2048});
  const bool overlaps = b.data < a.data + spanning && a.data < b.data + 2048;
  INFO("first allocation [", a.data, ", ", a.data + spanning, ")  second [", b.data, ", ", b.data + 2048, ")");
  CHECK_FALSE(overlaps);
}
