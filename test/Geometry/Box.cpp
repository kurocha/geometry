
#include <UnitTest/UnitTest.hpp>

#include <Geometry/Box.hpp>

namespace Geometry
{
	UnitTest::Suite BoxTestSuite {
		"Geometry::Box",

		{"Construction",
			[](UnitTest::Examiner & examiner) {
				Box<2> a(ZERO), b(IDENTITY), c(ZERO);
				examiner << "Zero box" << std::endl;
				examiner.check(a.min().equivalent(0));
				examiner << "Zero box" << std::endl;
				examiner.check(a.max().equivalent(0));

				examiner << "Identity box" << std::endl;
				examiner.check(b.min().equivalent(0));
				examiner << "Identity box" << std::endl;
				examiner.check(b.max().equivalent(1));

				examiner << "Zero box is degenerate" << std::endl;
				examiner.check(a.is_degenerate());
				examiner << "Identity box is normal" << std::endl;
				examiner.check(b.is_normal());
			}
		},

		{"Box-Point Intersection",
			[](UnitTest::Examiner & examiner) {
				Box<3> box1(vector(0.0, 0.0, 0.0), vector(1.0, 1.0, 1.0)),
				box2(vector(0.0, 0.0, 0.0), vector(10.0, 10.0, 10.0)),
				box3(vector(5.0, 5.0, 5.0), vector(10.0, 10.0, 10.0)),
				box4(vector(9.0, 9.0, 9.0), vector(10.0, 10.0, 10.0)),
				box5(vector(-5.0, 0.0, 0.0), vector(5.0, 10.0, 10.0)),
				box5a(vector(0.0, 0.0, 0.0), vector(5.0, 10.0, 10.0)),
				box6(vector(5.0, 0.0, 0.0), vector(10.0, 10.0, 10.0));

				examiner << "Contains point was correct" << std::endl;
				examiner.check(box1.contains_point(vector(0.0, 0.0, 0.0), true ));
				examiner << "Contains point was correct" << std::endl;
				examiner.check(!box1.contains_point(vector(0.0, 0.0, 0.0), false));
				examiner << "Contains point was correct" << std::endl;
				examiner.check(box1.contains_point(vector(1.0, 1.0, 0.5), true ));
				examiner << "Contains point was correct" << std::endl;
				examiner.check(!box1.contains_point(vector(1.0, 1.0, 0.5), false));
				examiner << "Contains point was correct" << std::endl;
				examiner.check(box1.contains_point(vector(1.0, 1.0, 1.0), true ));
				examiner << "Contains point was correct" << std::endl;
				examiner.check(!box1.contains_point(vector(1.0, 1.0, 1.0), false));
				examiner << "Contains point was correct" << std::endl;
				examiner.check(!box1.contains_point(vector(1.0, 2.0, 1.0), true ));
				examiner << "Contains point was correct" << std::endl;
				examiner.check(!box1.contains_point(vector(1.0, 1.0, 2.0), false));
				examiner << "Contains point was correct" << std::endl;
				examiner.check(!box1.contains_point(vector(3.0, 1.0, 0.0), false));
			}
		},

		{"Box-Box Intersection",
			[](UnitTest::Examiner & examiner) {
				Box<3> box1(vector(0.0, 0.0, 0.0), vector(1.0, 1.0, 1.0)),
					box2(vector(0.0, 0.0, 0.0), vector(10.0, 10.0, 10.0)),
					box3(vector(5.0, 5.0, 5.0), vector(10.0, 10.0, 10.0));

				examiner << "Intersection result was correct" << std::endl;
				examiner.check(box1.intersects_with(box2));
				examiner << "Intersection result was correct" << std::endl;
				examiner.check(!box1.intersects_with(box3));
			}
		},

		{"Orientation",
			[](UnitTest::Examiner & examiner) {
				Box<3> box1(vector(0.0, 0.0, 0.0), vector(1.0, 1.0, 1.0));
				
				Box<3> box1a(box1),
					box2(vector(0.0, 0.0, 0.0), vector(10.0, 10.0, 10.0)),
					box3(vector(5.0, 5.0, 5.0), vector(10.0, 10.0, 10.0)),
					box4(vector(9.0, 9.0, 9.0), vector(10.0, 10.0, 10.0));

				examiner << "Box orientation was correct" << std::endl;
				examiner.check(box2.orientation_of(box3).equivalent(1.0));
				box1a.align_within_super_box(box2, vector(1.0, 1.0, 1.0));
				examiner << "Box was aligned correctly" << std::endl;
				examiner.check(box1a.equivalent(box4));
				box1a.align_within_super_box(box2, vector(0.0, 0.0, 0.0));
				examiner << "Box was aligned correctly" << std::endl;
				examiner.check(box1.equivalent(box1a));

				// Is this desirable behaviour?
				Box<2> o1(ZERO, 10), o2(ZERO, 5), o3(20, 30);

				examiner.check(o1.orientation_of(o2) == Vec2(0, 0));
				examiner.check(o1.orientation_of(o3) == Vec2(1, 1));
			}
		},

		{"Subtraction",
			[](UnitTest::Examiner & examiner) {
				Box<3> box1(vector(0.0, 0.0, 0.0), vector(1.0, 1.0, 1.0)),
				box2(vector(0.0, 0.0, 0.0), vector(10.0, 10.0, 10.0)),
				box3(vector(5.0, 5.0, 5.0), vector(10.0, 10.0, 10.0)),
				box4(vector(9.0, 9.0, 9.0), vector(10.0, 10.0, 10.0)),
				box5(vector(-5.0, 0.0, 0.0), vector(5.0, 10.0, 10.0)),
				box5a(vector(0.0, 0.0, 0.0), vector(5.0, 10.0, 10.0)),
				box6(vector(5.0, 0.0, 0.0), vector(10.0, 10.0, 10.0));
				
				Box<3> box2a(box2);
				// Subtract left half
				box2a.subtract_in_order(box5, vector<unsigned>(0, 1, 2));

				// We should have the right half
				examiner << "Box was subtracted correctly" << std::endl;
				examiner.check(box2a == box6);

				Box<3> box2b(box2);
				// Subtract right half
				box2b.subtract_in_order(box6, vector<unsigned>(0, 1, 2));

				// We should have the left half
				examiner << "Box was subtracted correctly" << std::endl;
				examiner.check(box2b == box5a);

				Box<3> c1(Vec3(0, 0, 0), Vec3(50, 10, 10)),
				c2(Vec3(5, 5, 0), Vec3(15, 15, 15)),
				c3(Vec3(5, 10, 0), Vec3(15, 15, 15));

				Box<3> c2a(c2);

				c2a.subtract_in_order(c1, vector<unsigned>(0, 1, 2));
				examiner << "Box was subtracted correctly" << std::endl;
				examiner.check(c2a == c3);
			}
		},
		
		// {"it can read and write to string",
		// 	[](UnitTest::Examiner & examiner) {
		// 		std::stringstream input("{{1,2},{3,4}}"), output;
		// 		
		// 		Box<2, unsigned> box;
		// 		input >> box;
		// 		
		// 		examiner.expect(box) == Box<2, unsigned>{{1,2},{3,4}};
		// 		
		// 		output << box;
		// 		
		// 		examiner.expect(output.str()) == "{{1,2},{3,4}}";
		// 	}
		// },
	};
}
