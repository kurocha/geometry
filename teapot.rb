# Teapot v2.2.0 configuration generated at 2017-11-04 20:06:32 +1300

required_version "2.0"

# Project Metadata

define_project "geometry" do |project|
	project.title = "Geometry"
	project.summary = 'A brief one line summary of the project.'
	
	project.description = <<-EOF
		Geometry description.
	EOF
	
	project.license = 'MIT License'
	
	project.add_author 'Samuel Williams', email: 'samuel.williams@oriontransfer.co.nz'
	# project.website = 'http://Geometry.com/'
	
	project.version = '0.1.0'
end

# Build Targets

define_target 'geometry-library' do |target|
	target.depends 'Library/Numerics', public: true
	target.depends 'Language/C++14'
	
	target.provides 'Library/Geometry'do
		source_root = target.package.path + 'source'
		
		library_path = build static_library: 'Geometry', source_files: source_root.glob('Geometry/**/*.cpp')
		
		append linkflags library_path
		append header_search_paths source_root
	end
end

define_target 'geometry-test' do |target|
	target.depends 'Library/UnitTest'
	
	target.depends 'Language/C++14'
	target.depends 'Library/Geometry'
	
	target.provides 'Test/Geometry' do |*arguments|
		test_root = target.package.path + 'test'
		
		run source_files: test_root.glob('Geometry/**/*.cpp'), arguments: arguments
	end
end

# Configurations

define_configuration 'development' do |configuration|
	configuration[:source] = "https://github.com/kurocha"
	configuration.import "geometry"
	
	# Provides all the build related infrastructure:
	configuration.require 'platforms'
	
	# Provides unit testing infrastructure and generators:
	configuration.require 'unit-test'
	
	# Provides some useful C++ generators:
	configuration.require 'generate-cpp-class'
	
	configuration.require "generate-project"
	configuration.require "generate-travis"
end

define_configuration "geometry" do |configuration|
	configuration.public!
	
	configuration.require "numerics"
end
