
#include <UnitTest/UnitTest.hpp>

#include <Geometry/Tree.hpp>

namespace Geometry
{
	UnitTest::Suite TreeTestSuite {
		"Geometry::Tree",

		{"Construction",
			[](UnitTest::Examiner & examiner) {
				Tree<Quadrants, Box2> tree(0, 20);
				
				examiner << "Tree has correct size." << std::endl;
				examiner.check_equal(tree.top()->origin(), Vec2{0, 0});
				examiner.check_equal(tree.top()->size(), Vec2{20, 20});
				examiner.check_equal(tree.top()->level(), 0);
				
				for (unsigned i = 0; i < 20; i += 1)
					tree.insert(Box2{i, i+1});
				
				examiner << "The tree has been redistributed." << std::endl;
				examiner.check_equal(tree.top()->objects().size(), 0);
				examiner.check_equal(tree.top()->child(Quadrants::BottomLeft)->objects().size(), 10);
				examiner.check_equal(tree.top()->child(Quadrants::TopRight)->objects().size(), 10);
				examiner.check_equal(tree.top()->child(Quadrants::BottomRight)->objects().size(), 0);
				examiner.check_equal(tree.top()->child(Quadrants::TopLeft)->objects().size(), 0);
			}
		},
	};
}
