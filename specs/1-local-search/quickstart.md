# Quickstart Guide: Local Search for Docusaurus Documentation

This guide outlines how to quickly set up and verify the local search functionality in the Docusaurus documentation site.

## Prerequisites

-   Node.js (LTS version) and npm/yarn installed.
-   A local clone of the project repository, with the `1-local-search` branch checked out.
-   All necessary dependencies installed (`npm install` or `yarn install` in `docusaurus-book/`).

## How to Verify Local Search Functionality

### 1. Run the Docusaurus Development Server

Navigate to the `docusaurus-book` directory and start the development server:

```bash
cd docusaurus-book
npm run start
# or
yarn start
```

This will open your Docusaurus site in your web browser, typically at `http://localhost:3000`.

### 2. Test Online Search

1.  **Navigate**: Browse to any documentation page.
2.  **Locate Search Bar**: Find the search bar/icon (usually in the navigation header).
3.  **Enter Query**: Type a relevant keyword (e.g., "ROS 2", "kinematics", "simulation") that you know exists in the documentation content.
4.  **Verify Results**:
    *   A list of search results should appear dynamically.
    *   Results should include a title and a snippet of the matching content.
    *   Click on a search result to ensure it navigates to the correct page/section.
    *   Test queries for content from different chapters and subchapters.
    *   Test queries for terms expected to yield no results (e.g., "xyzabc") and verify "No results found" message.

### 3. Test Offline Search (PWA Functionality)

1.  **Ensure Service Worker is Registered**: While the Docusaurus site is running in your browser (from step 1), open your browser's developer tools (F12 or Ctrl+Shift+I).
2.  **Go to Application Tab**: In the developer tools, navigate to the "Application" tab.
3.  **Service Workers**: In the left sidebar, select "Service Workers". You should see a service worker registered for your site (e.g., `service-worker.js`). Ensure it is "activated and running".
4.  **Simulate Offline**: In the "Service Workers" pane, check the "Offline" checkbox (or disconnect your internet connection).
5.  **Refresh Page**: Refresh your Docusaurus site tab. It should still load, served from cache.
6.  **Perform Offline Search**: Use the search bar as described in "2. Test Online Search" with the network still offline.
7.  **Verify Offline Results**:
    *   The search functionality should still work, providing results from the locally cached index.
    *   Click on results to verify navigation to cached pages.
8.  **Go Back Online**: Uncheck the "Offline" checkbox in developer tools or reconnect your internet.

### 4. Build and Deploy Verification (Conceptual)

While full deployment to GitHub Pages is outside this quickstart, you should conceptually understand that the `npm run build` command (or `yarn build`) will generate the static assets, including the search index and service worker, ready for deployment to a static hosting service like GitHub Pages. After deployment, the online and offline search functionalities should behave as tested locally.
