import { withMermaid } from "vitepress-plugin-mermaid";
import rosidl from "./ros-idl";

// https://vitepress.dev/reference/site-config
export default withMermaid({
  title: "OpenMowerNext",
  description: "Just another software stack for OpenMower lawn mower robot, but this time with ROS2!",
  base: "/OpenMowerNext/",
  ignoreDeadLinks: [
    // ignore all localhost links
    /^https?:\/\/localhost/,
  ],
  markdown: {
    shikiSetup: (shiki) => {
      shiki.loadLanguage(rosidl);
    },
  },
  themeConfig: {
    search: {
      provider: 'local',
    },
    // https://vitepress.dev/reference/default-theme-config
    nav: [
      { text: 'Home', link: '/' },
      { text: 'Documentation', link: '/getting-started' }
    ],

    sidebar: [
      {
        text: 'Getting started',
        link: '/getting-started',
        items: [
          { text: 'Roadmap', link: '/roadmap' },
        ],
      },
      {
        text: 'Architecture',
        items: [
          { text: 'ROS workspace', link: '/architecture/ros-workspace' },
          { text: 'Mainboard firmware', link: '/architecture/omros2-firmware' },
          { text: 'Robot localization', link: '/architecture/localization' },
          { text: 'Map server', link: '/architecture/map-server' },
          { text: 'Map recorder', link: '/architecture/map-recorder' },
          { text: 'Docking helper', link: '/architecture/docking-helper' },
          { text: 'Sim node', link: '/architecture/sim-node' },
        ]
      },
      {
        text: 'Contributing guide',
        link: '/contributing',
        items: [
            {
              text: 'Devcontainer',
              link: '/devcontainer',
              items: [
                { text: 'CLion alternative', link: '/clion-env' },
              ],
            },
            { text: 'Visualisation', link: '/visualisation' },
            { text: 'Simulator', link: '/simulator' },
            { text: 'ROS MCP', link: '/ros-mcp' },
        ],
      },
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/jkaflik/OpenMowerNext' },
      { icon: 'discord', link: 'https://discord.gg/jE7QNaSxW7' },
    ]
  }
})
