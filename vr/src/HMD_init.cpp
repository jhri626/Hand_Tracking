#include <Windows.h>
#include <glad/glad.h>    
#include <GL/gl.h>
#include <OpenXRProvider.h>  
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <iostream>
#include <cstring>
#include <spdlog/spdlog.h>
#include <thread>
#include "utils.h"
#include "HMD.h"
#include "HMD_number.h"




bool HMD::initSystem() 
{
    // Prepare the system information structure for a head-mounted display.
    XrSystemGetInfo systemInfo{ XR_TYPE_SYSTEM_GET_INFO };
    systemInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;

    // Retrieve the system ID for the OpenXR instance.
    XrResult result = xrGetSystem(xrInstance, &systemInfo, &xrSystemId);
    if (XR_FAILED(result)) {
        std::cout << "[error] Failed to get OpenXR system!" << std::endl;
        return false;
    }

    // Check if the obtained system ID is valid.
    if (xrSystemId == XR_NULL_SYSTEM_ID) {
        std::cout << "[error] OpenXR system ID is invalid!" << std::endl;
        return false;
    }
    return true;
}


bool HMD::initOpenGL() {
    // If the window does not exist, create it.
    if (!hWnd && !CreateRenderWindow(hWnd)) {
        std::cerr << "No window" << std::endl;
        return false;
    }
    // Obtain the device context from the window.
    hDC = GetDC(hWnd);
    if (hDC == nullptr) {
        std::cerr << "Failed to get Device Context." << std::endl;
        return false;
    }

    // Define the pixel format descriptor.
    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize      = sizeof(PIXELFORMATDESCRIPTOR);
    pfd.nVersion   = 1;
    pfd.dwFlags    = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    pfd.iLayerType = PFD_MAIN_PLANE;

    // Choose the best pixel format that matches the descriptor.
    int pixelFormat = ChoosePixelFormat(hDC, &pfd);
    if (pixelFormat == 0) {
        std::cerr << "Failed to choose a pixel format." << std::endl;
        return false;
    }
    // Set the chosen pixel format for the device context.
    if (!SetPixelFormat(hDC, pixelFormat, &pfd)) {
        std::cerr << "Failed to set the pixel format."<<GetLastError() << std::endl;
        return false;
    }

    // Create the OpenGL rendering context.
    hGLRC = wglCreateContext(hDC);
    if (hGLRC == nullptr) {
        std::cerr << "Failed to create OpenGL Rendering Context." << std::endl;
        return false;
    }
    std::cerr << "OpenGL context created successfully. HGLRC: " << hGLRC << std::endl;

    // Make the created context current.
    if (!wglMakeCurrent(hDC, hGLRC)) {
        std::cerr << "Failed to make OpenGL context current." << std::endl;
        return false;
    }
    
    return true;
}

bool HMD::CreateOpenXRInstanceAndSession() {
    // List of required extension names.
    const char* extensionNames[] = {
        XR_EXT_HAND_TRACKING_EXTENSION_NAME,      // Hand tracking extension
        XR_KHR_OPENGL_ENABLE_EXTENSION_NAME        // OpenGL enable extension
        // XR_KHR_COMPOSITION_LAYER_COLOR_EXTENSION_NAME // for frame debug remove it later
    };

    if (!gladLoadGL()) {
        std::cerr << "[error] Failed to load GL functions\n";
        return false;
    }

    // Setup the OpenXR instance creation info.
    XrInstanceCreateInfo xrInstanceCreateInfo = { XR_TYPE_INSTANCE_CREATE_INFO };
    std::strcpy(xrInstanceCreateInfo.applicationInfo.applicationName, "ViveFocusHandTracking");
    xrInstanceCreateInfo.applicationInfo.applicationVersion = 1;
    std::strcpy(xrInstanceCreateInfo.applicationInfo.engineName, "CustomEngine");
    xrInstanceCreateInfo.applicationInfo.engineVersion = 1;
    xrInstanceCreateInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    xrInstanceCreateInfo.enabledExtensionCount = sizeof(extensionNames) / sizeof(extensionNames[0]);
    xrInstanceCreateInfo.enabledExtensionNames = extensionNames;

    // Create the OpenXR instance.
    XrResult result = xrCreateInstance(&xrInstanceCreateInfo, &xrInstance);
    if (result != XR_SUCCESS) {
        std::cerr << "Failed to create OpenXR instance." << std::endl;
        return false;
    }

    // Initialize the OpenXR system.
    if (!initSystem()) {
        std::cerr << "Failed to initialize OpenXR system!" << std::endl;
        return false;
    }

    // Retrieve the function pointer for obtaining OpenGL graphics requirements.
    PFN_xrGetOpenGLGraphicsRequirementsKHR pfn_xrGetOpenGLGraphicsRequirementsKHR = nullptr;
    result = xrGetInstanceProcAddr(
        xrInstance, "xrGetOpenGLGraphicsRequirementsKHR",
        reinterpret_cast<PFN_xrVoidFunction*>(&pfn_xrGetOpenGLGraphicsRequirementsKHR)
    );
    if (result != XR_SUCCESS || pfn_xrGetOpenGLGraphicsRequirementsKHR == nullptr) {
        std::cerr << "Failed to get xrGetOpenGLGraphicsRequirementsKHR function pointer." << std::endl;
        std::cerr << "Error: " << GetLastError() << std::endl;
        std::cerr << "Result: " << result << std::endl;
        return false;
    }

    // Query the OpenGL graphics requirements.
    XrGraphicsRequirementsOpenGLKHR graphicsRequirements = { XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR };
    result = pfn_xrGetOpenGLGraphicsRequirementsKHR(xrInstance, xrSystemId, &graphicsRequirements);
    if (result != XR_SUCCESS) {
        std::cerr << "Failed to get OpenGL graphics requirements. Error: " << result << std::endl;
        return false;
    }

    // Ensure that the OpenGL context is correctly initialized.
    if (hDC == nullptr || hGLRC == nullptr) {
        std::cerr << "Error: OpenGL context is not initialized properly." << std::endl;
        return false;
    }

    // Setup session creation info and graphics binding.
    XrSessionCreateInfo xrSessionCreateInfo = { XR_TYPE_SESSION_CREATE_INFO };
    xrSessionCreateInfo.systemId = xrSystemId;
    
    XrGraphicsBindingOpenGLWin32KHR graphicsBinding{};
    graphicsBinding.type = XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR;
    graphicsBinding.hDC = hDC;
    graphicsBinding.hGLRC = hGLRC;
    xrSessionCreateInfo.next = &graphicsBinding;

    // Debug prints for context values.
    std::cerr << "hDC value: " << hDC << std::endl;
    std::cerr << "hGLRC value: " << hGLRC << std::endl;

    if (hDC == nullptr || hGLRC == nullptr) {
        std::cerr << "Error: OpenGL context is not initialized properly." << std::endl;
        return false;
    }

    std::cerr << "xrSessionCreateInfo.systemId: " << xrSessionCreateInfo.systemId << std::endl;
    std::cerr << "xrSessionCreateInfo.next: " << xrSessionCreateInfo.next << std::endl;

    // Create the OpenXR session.
    result = xrCreateSession(xrInstance, &xrSessionCreateInfo, &xrSession);
    if (result != XR_SUCCESS) {
        std::cerr << "Failed to create OpenXR session. Error: " << result << std::endl;
        return false;
    }
    return true;
}

bool HMD::CreateReferenceSpace(XrReferenceSpaceType type, XrSpace &outSpace){
    // Create a reference space for the session.
    XrReferenceSpaceCreateInfo referenceSpaceCreateInfo = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    referenceSpaceCreateInfo.referenceSpaceType = type;
    referenceSpaceCreateInfo.poseInReferenceSpace.orientation.w = 1.0f;
    referenceSpaceCreateInfo.poseInReferenceSpace.position = { 0.0f, 0.0f, 0.0f };

    XrResult result = xrCreateReferenceSpace(xrSession, &referenceSpaceCreateInfo, &outSpace);
    if (result != XR_SUCCESS) {
        std::cerr << "Failed to create OpenXR space." << std::endl;
        return false;
    }
    return true;
}

bool HMD::beginOpenXRSession() {

    if (xrInstance == XR_NULL_HANDLE) {
        std::cerr << "[error] xrInstance is NULL! Cannot start session." << std::endl;
        return false;
    }
    if (xrSession == XR_NULL_HANDLE) {
        std::cerr << "[error] xrSession is NULL! Cannot start session." << std::endl;
        return false;
    }
    XrResult result;
    XrInstanceProperties instanceProperties{ XR_TYPE_INSTANCE_PROPERTIES };
    result = xrGetInstanceProperties(xrInstance, &instanceProperties);
    if (XR_FAILED(result)) {
        std::cerr << "[error] xrInstance is invalid just before calling xrPollEvent(). Error code: " << result << std::endl;
        return false;
    }

    bool running = true;
    while (running) {
        XrEventDataBuffer event{XR_TYPE_EVENT_DATA_BUFFER};
        while (xrPollEvent(xrInstance, &event) == XR_SUCCESS) {
            std::cout << "Session state changed: " << currentSessionState << std::endl;
            if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                auto* ev = reinterpret_cast<XrEventDataSessionStateChanged*>(&event);
                currentSessionState = ev->state;
                std::cout << "Session state changed: " << currentSessionState << std::endl;
                if (currentSessionState == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo si{XR_TYPE_SESSION_BEGIN_INFO}; si.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                    XrResult res = xrBeginSession(xrSession, &si);
                    if (XR_FAILED(res)) {
                        std::cerr << "[error] Failed to begin OpenXR session! Error code: " << res << std::endl;
                        return false;
                    }
                    std::cout << "Session started. Creating reference space..." << std::endl;
                    CreateReferenceSpace(XR_REFERENCE_SPACE_TYPE_LOCAL, worldSpace);
                    CreateReferenceSpace(XR_REFERENCE_SPACE_TYPE_VIEW, hmdSpace);
                    std::cout << "Reference space created." << std::endl;
                    return true;
                }
                if (currentSessionState == XR_SESSION_STATE_STOPPING || currentSessionState == XR_SESSION_STATE_EXITING) running = false;
            }
            event = {XR_TYPE_EVENT_DATA_BUFFER};
        }
        
    }

    // session not ready
    std::cerr << "[error] OpenXR session is not ready!" << std::endl;
    return false;
}

bool HMD::CreateSwapchain() {

uint32_t formatCount = 0;
xrEnumerateSwapchainFormats(xrSession, 0, &formatCount, nullptr);
std::vector<int64_t> formats(formatCount);
xrEnumerateSwapchainFormats(xrSession, formatCount, &formatCount, formats.data());

// Choose a compatible format
int64_t chosenFormat = formats[0]; // Default to the first available format
for (int64_t format : formats) {
    if (format == GL_RGBA8 || format == GL_SRGB8_ALPHA8) { 
        chosenFormat = format;
        break;
    }
}


XrSwapchainCreateInfo swapchainCreateInfo = {};
swapchainCreateInfo.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
swapchainCreateInfo.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
swapchainCreateInfo.format = chosenFormat ;
swapchainCreateInfo.sampleCount = 1;
swapchainCreateInfo.width = HMDVariable::SWAPCHAIN_WIDTH;
swapchainCreateInfo.height = HMDVariable::SWAPCHAIN_HEIGHT;
swapchainCreateInfo.faceCount = 1;
swapchainCreateInfo.arraySize = 1;
swapchainCreateInfo.mipCount = 1;

XrResult result = xrCreateSwapchain(xrSession, &swapchainCreateInfo, &xrSwapchain);
if (XR_FAILED(result)) {
    std::cerr << "Failed to create swapchain"<< result << std::endl;
    return XR_NULL_HANDLE;
}

std::cout << "Swapchain created successfully." << std::endl;

uint32_t imageCount = 0;
xrEnumerateSwapchainImages(xrSwapchain, 0, &imageCount, nullptr);


swapchainImages.resize(imageCount, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR});

result = xrEnumerateSwapchainImages(
    xrSwapchain,
    imageCount,
    &imageCount,
    reinterpret_cast<XrSwapchainImageBaseHeader*>(swapchainImages.data())
);
if (XR_FAILED(result)) {
    std::cerr << "Failed to enumerate swapchain images: " << result << std::endl;
    return false;
}

std::cout << "Swapchain created successfully with " << imageCount << " images." << std::endl;
return true;
}
