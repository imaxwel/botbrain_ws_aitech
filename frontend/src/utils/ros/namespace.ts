const DEFAULT_ROS_NAMESPACE = 'g1_robot';

export function getRosNamespace(): string {
  const namespace = process.env.NEXT_PUBLIC_ROS_NAMESPACE || DEFAULT_ROS_NAMESPACE;
  return namespace.replace(/^\/+|\/+$/g, '');
}

export function getNamespacedRosTopic(topic: string): string {
  const cleanTopic = topic.replace(/^\/+/g, '');
  const namespace = getRosNamespace();

  return namespace ? `/${namespace}/${cleanTopic}` : `/${cleanTopic}`;
}
